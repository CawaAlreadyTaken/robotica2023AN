#ifndef __PATH_FINDING__
#define __PATH_FINDING__

#include <iostream>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <unistd.h>
#include <cmath>

using namespace std;
using namespace Eigen;

using namespace std;
const double pi = 3.1415926;

typedef vector<vector<double>> Envmap;
typedef vector<pair<int, int>> Path;
typedef pair<int, int> Node;
typedef vector<vector<Node>> Fathers;
typedef vector<vector<Matrix<double, 6, 1>>> Jointmap;

double euclidean_distance(Node u, Node v);
double manhattan_distance(Node u, Node v);
Vector3d linear_interpol(Vector3d pi, Vector3d pf, double t);
Vector3d trapezoidal_velocity(Vector3d pi, Vector3d pf, double t);
Vector3d linear_velocity(Vector3d pi, Vector3d pf);
double close_to_collisions(Node u, Node v, Matrix<double, 6, 1> joints);
vector<Node> get_lines(Path path);
Path a_star(Node start, Node end, Envmap& gscores, Envmap& hscores, Envmap& fscores, Fathers& fathers, Jointmap& jointmap, double(*heuristics)(Node, Node), double(*collisions)(Node, Node, Matrix<double, 6, 1>));
void init(Node start, Node end, Envmap& gscores, Envmap& hscores, Envmap& fscores, Fathers& fathers);
Node get_closest_node(Vector3d wcoords);
#endif
