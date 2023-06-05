#include "path_finding.h"

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
	}

	if(t <= 0.25) {
		return acc * t;
	}

	if(t > 0.25 && t <= 0.75) {
		return vel;
	}

	if(t > 0.75 && t < 1) {
		return vel - acc * (t - 0.75);
	}

	return Vector3d::Zero();
}

Vector3d linear_velocity(Vector3d pi, Vector3d pf) {
	return (pf - pi);
}

Matrix<double, 6, 1> get_joints(Node start, Node end, Jointmap& jointmap, double gripper = 0, double k=0.1, double k_rpy=0.1) {

	Matrix<double, 6, 1> q0;
	Vector3d v;
	Matrix3d K = Matrix3d::Identity() * k;
	Matrix3d K_rpy = Matrix3d::Identity() * k_rpy;

	// read joints from jointmap, this value should be defined according to my logic
	q0 = jointmap[start.first][start.second];

	Vector3d start_rcoords((double)start.first/SCALE, (double)start.second/SCALE, SET_HEIGHT);
	Vector3d end_rcoords((double)end.first/SCALE, (double)end.second/SCALE, SET_HEIGHT);

	start_rcoords = diffKinPF.fromWorldToUr5(start_rcoords);
	end_rcoords = diffKinPF.fromWorldToUr5(end_rcoords);

	//debugging
	// cout << "start wcoords: " << start.first << ", " << start.second << endl;
	// cout << "start jcoords" << endl << jointmap[start.first][start.second] << endl << endl;

	double Dt = (double)1 / (loop_frequency * TIME_FOR_MOVING);
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
	Vector3d u_vector((double)u.first/SCALE, (double)u.second/SCALE, SET_HEIGHT);
	Vector3d v_vector((double)v.first/SCALE, (double)v.second/SCALE, SET_HEIGHT);
	return diffKinPF.close_to_collisions(u_vector, v_vector, joints);
}

vector<Node> neighbors(Node current) {
	int i[8] = {1, 1, 1, 0, 0, -1, -1, -1};
	int j[8] = {1, 0, -1, 1, -1, 1, 0, -1};
	vector<Node> res;

	for(int k = 0; k < DIM; k++) {
		if(current.first + i[k] > DIM || current.first + i[k] < 0 || current.second + j[k] > DIM || current.second + j[k] < 0) continue;
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
    vector<Node> res;
    res.push_back(path[0]);

    enum Direction {HORIZONTAL, VERTICAL, OBLIQUE};
    Direction direction;

    // get the direction of the 2 initial points
    if (path[0].first == path[1].first) direction = VERTICAL;
    else if (path[0].second == path[1].second) direction = HORIZONTAL;
    else direction = OBLIQUE;

    for (int i = 2; i < path.size(); i++) {

        if(i == path.size()-1) {   
            if(path[i].first == path[i-1].first && direction != VERTICAL) {
                res.push_back(path[i-1]);
                res.push_back(path[i]);
            }

            if(path[i].second == path[i-1].second && direction != HORIZONTAL){
              res.push_back(path[i-1]);
              res.push_back(path[i]);  
            } 

            if(manhattan_distance(path[i], path[i-1]) > 1 && direction != OBLIQUE) {
                res.push_back(path[i-1]);
                res.push_back(path[i]);
            }
            break;
        }

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


        // does not handle last element of path correctly
    }

    if(res[res.size()-1] == res[0]) res.push_back(path[path.size() - 1]);
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
	set<Node> open_set;
	Node current;
	int counter;

	open_set.insert(start);

	while(!open_set.empty()) {
		current = get_min_fscore(open_set, fscores);

		open_set.erase(current);

		if(current == end)
			return reconstruct_path(end, start, fathers);	

		for(auto n: neighbors(current)) {
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

	return {{-1, -1}};
}

void init(Node start, Node end, Envmap& gscores, Envmap& hscores, Envmap& fscores, Fathers& fathers){
	for(int i = 0; i < DIM; i++) {
		for(int j = 0; j < DIM; j++) {
			fathers[i][j] = {-1, -1};
			gscores[i][j] = HIGH_COST;
			hscores[i][j] = manhattan_distance({i, j}, end);
			fscores[i][j] = gscores[i][j] + manhattan_distance({i, j}, end);
		}
	}

	gscores[start.first][start.second] = 0;
	fscores[start.first][start.second] = hscores[start.first][start.second];
	
}

Node get_closest_node(Vector3d wcoords) {
	return Node(round(wcoords[0] * 10) / 10, round(wcoords[1] * 10) / 10);
}
