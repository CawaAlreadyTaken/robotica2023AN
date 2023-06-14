#include "path_finding.h"
#include "differentialKin.cpp"

#define DIM 20
#define SET_HEIGHT 1.174
#define INCREMENT 0.1
#define HIGH_COST 100000

bool operator == (const Node& u, const Node& v) {
	return u.first == v.first && u.second == v.second;
}

bool operator != (const Node& u, const Node& v) {
	return ! (u == v);
}

double euclidean_distance(Node u, Node v) {
	return sqrt(pow(u.first - v.first, 2) + pow(u.second - v.second, 2));
}

double manhattan_distance(Node u, Node v) {
	return abs(u.first - v.first) + abs(u.second - v.second);
}

void print_matrix(const Jointmap& jointmap) {
	for(auto line: jointmap) {
		for(auto e: line) {
			cout << e << endl;
		}
		cout << endl;
	}
}

Vector3d linear_interpol(Vector3d pi, Vector3d pf, double t)
{
	return t * (pf) + (1 - t) * pi;
}

Vector3d trapezoidal_velocity(Vector3d pi, Vector3d pf, double t) {
  Vector3d acc = (pf - pi) * 16 / 3;
  Vector3d vel = acc/4;

  if(t >= 1) {
    return Vector3d(0, 0, 0);
  } else if(t <= 0.25) {
    return acc * t;
  } else if(t > 0.25 && t <= 0.75) {
    return vel;
  } else {
    return vel - acc * (t - 0.75);
  }
}

Vector3d linear_velocity(Vector3d pi, Vector3d pf) {
	return (pf - pi);

}

Matrix<double, 6, 1> get_joints(Node start, Node end, Jointmap& jointmap, double gripper = 0, double k=0.1, double k_rpy=0.1) {

	DifferentialKinematic diffKinPF = DifferentialKinematic();

	Matrix<double, 6, 1> q0;
	Vector3d v;
	Matrix3d K = Matrix3d::Identity() * k;
	Matrix3d K_rpy = Matrix3d::Identity() * k_rpy;

	// read joints from jointmap, this value should be defined according to my logic
	q0 = jointmap[start.first][start.second];

	Vector3d start_rcoords((double)start.first/DIM, (double)start.second/DIM, SET_HEIGHT);
	Vector3d end_rcoords((double)end.first/DIM, (double)end.second/DIM, SET_HEIGHT);

	start_rcoords = diffKinPF.fromWorldToUr5(start_rcoords);
	end_rcoords = diffKinPF.fromWorldToUr5(end_rcoords);

	double nsamples = 6;

	//debugging
	// cout << "start wcoords: " << start.first << ", " << start.second << endl;
	// cout << "start jcoords" << endl << jointmap[start.first][start.second] << endl << endl;

	//double Dt = (double)1 / (loop_frequency * TIME_FOR_MOVING);
	double Dt = (double)1 / (nsamples);
	double t = Dt;
	while(t<=1) {
		q0 += diffKinPF.Qdot(q0, linear_interpol(start_rcoords, end_rcoords, t), trapezoidal_velocity(start_rcoords, end_rcoords, t), Vector3d(0, 0, pi), Vector3d::Zero(), K, K_rpy) * Dt;
		t+=Dt;
	}

	// cout << "end wcoords: " << end.first << ", " << end.second << endl;
	// cout << "end jcoords" << endl << q0 << endl << endl;

	return q0;
}

double close_to_collisions(Node u, Node v, Matrix<double, 6, 1> joints) {
	DifferentialKinematic diffKinPF = DifferentialKinematic();

	Vector3d u_vector((double)u.first/DIM, (double)u.second/DIM, SET_HEIGHT);
	Vector3d v_vector((double)v.first/DIM, (double)v.second/DIM, SET_HEIGHT);
	return diffKinPF.close_to_collisions(u_vector, v_vector, joints);
}


vector<Node> neighbors(Node current) {
	int i[8] = {1, 1, 1, 0, 0, -1, -1, -1};
	int j[8] = {1, 0, -1, 1, -1, 1, 0, -1};
	vector<Node> res;

	for(int k = 0; k < 8; k++) {
		if(current.first + i[k] >= DIM || current.first + i[k] < 0 || current.second + j[k] >= DIM || current.second + j[k] < 0) continue;
		res.push_back({current.first + i[k], current.second + j[k]});
	}

	return res;
}

Path reconstruct_path(Node end, Node start, Fathers fathers) {
	Node t = end;
	Path path;

	while(t != start) {
		path.push_back(t);
		t = fathers[t.first][t.second];
	}

	reverse(path.begin(), path.end());
	return path;
}

vector<Node> get_lines(Path path) {
	cout << "[*] Path travels through: " << path.size() << " cells" << endl;
    vector<Node> res;
    res.push_back(path[0]);

    enum Direction {HORIZONTAL, VERTICAL, OBLIQUE};
    Direction direction;

    // get the direction of the 2 initial points
    if (path[0].first == path[1].first) direction = VERTICAL;
    else if (path[0].second == path[1].second) direction = HORIZONTAL;
    else direction = OBLIQUE;

    for (int i = 2; i < path.size()-1; i++) {

        if(path[i].first == path[i-1].first && direction != VERTICAL){
            direction = VERTICAL;
            res.push_back(path[i-1]);
        }

        if(path[i].second == path[i-1].second && direction != HORIZONTAL) {
            direction = HORIZONTAL;
            res.push_back(path[i-1]);
        }

        if(manhattan_distance(path[i], path[i - 1]) > 1 && direction != OBLIQUE) {
            direction = OBLIQUE;
            res.push_back(path[i-1]);
        }
    }

	cout << "[*] Number of direction changes: " << res.size()-1 << endl;
    return res;
}

Node get_min_fscore(set<Node> set, Envmap fscores) {
	double MIN = (double)INT_MAX;
	Node res;

	for(auto s: set) {
		if(fscores[s.first][s.second] < MIN) {
			MIN = fscores[s.first][s.second];
			res = s;
		}
	}

	return res;
}

Path a_star(Node start, Node end, Envmap& gscores, Envmap& hscores, Envmap& fscores, Fathers& fathers, Jointmap& jointmap, double(*heuristics)(Node, Node), double(*collisions)(Node, Node, Matrix<double, 6, 1>)) {
	cout << "[*] A-star started" << endl << "start: " << start.first << ", " << start.second << endl << "end: " << end.first << ", " << end.second << endl << endl;
	set<Node> open_set;
	Node current;
	int counter;
	open_set.insert(start);
	while(!open_set.empty()) {
		current = get_min_fscore(open_set, fscores);
		open_set.erase(current);

		if(current == end) {
			cout << "[*] A-star found a path" << endl;
			cout << "[*] Cost of the found path: " << gscores[end.first][end.second] << endl;
			cout << "[*] fscores:" << endl;
			for (int i = 0; i < fscores.size(); i++) {
				for (int j = 0; j < fscores[i].size(); j++) {
					if (i == end.first && j == end.second)
						cout << "E" << fscores[i][j] << "E" << "\t";
					else if (i == start.first && j == start.second)
						cout << "S" << fscores[i][j] << "S" << "\t";
					else
						cout << fscores[i][j] << "\t";
				}
				cout << endl;
			}

			auto res = reconstruct_path(end, start, fathers);
			cout << "[*] Path length: " << res.size() << endl;
			cout << "[*] Path:" << endl;
			for (auto e: res) {
				cout << e.first << ", " << e.second << endl;
			}
			return res;
		}
		auto nb = neighbors(current);
		//cout << "neighbors number: " << nb.size() << endl;
		for(auto n: nb) {
			double tentative_score = gscores[current.first][current.second] + heuristics(current, end) + collisions(current, n, jointmap[current.first][current.second]);
			if(tentative_score < gscores[n.first][n.second]) {
				gscores[n.first][n.second] = tentative_score;
				fscores[n.first][n.second] = tentative_score + hscores[n.first][n.second];
				fathers[n.first][n.second] = current;
				jointmap[n.first][n.second] = get_joints(current, n, jointmap);
				if(open_set.find(n) == open_set.end()) open_set.insert(n);
			}
		}
	}

	cout << "[*] A-star did not find a path" << endl;
	return {{-1, -1}};
}

void init(Node start, Node end, Envmap& gscores, Envmap& hscores, Envmap& fscores, Fathers& fathers){
	for(int i = 0; i < DIM; i++) {
		for(int j = 0; j < DIM; j++) {
			fathers[i][j] = {-1, -1};
			gscores[i][j] = HIGH_COST;
			hscores[i][j] = euclidean_distance({i, j}, end);
			fscores[i][j] = gscores[i][j] + hscores[i][j];
		}
	}

	gscores[start.first][start.second] = 0;
	fscores[start.first][start.second] = hscores[start.first][start.second];
	
}

Node get_closest_node(Vector3d wcoords) {
	return Node((double)round(wcoords[0] * DIM), (double)round(wcoords[1] * DIM));
}