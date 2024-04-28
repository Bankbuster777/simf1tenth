#include "scan_matching_skeleton/transform.h"
#include <cmath>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <complex>
#include <unsupported/Eigen/Polynomials>

using namespace std;


void transformPoints(const vector<Point>& points, Transform& t, vector<Point>& transformed_points) {
  transformed_points.clear();
  for (int i = 0; i < points.size(); i++) {
    transformed_points.push_back(t.apply(points[i]));
    //printf("%f %transformed_points.back().r, transformed_points.back().theta);
  }
}

// returns the largest real root to ax^3 + bx^2 + cx + d = 0
complex<float> get_cubic_root(float a, float b, float c, float d) {
  // std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<<";"<<std::endl;
  // Reduce to depressed cubic
  float p = c/a - b*b/(3*a*a);
  float q = 2*b*b*b/(27*a*a*a) + d/a - b*c/(3*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;

  complex<float> xi(-.5, sqrt(3)/2);

  complex<float> inside = sqrt(q*q/4 + p*p*p/27);

  complex<float> root;

  for (float k = 0; k < 3; ++k) {
    // get root for 3 possible values of k
    root = -b/(3*a) + pow(xi, k) * pow(-q/2.f + inside, 1.f/3.f) + pow(xi, 2.f*k) * pow(-q/2.f - inside, 1.f/3.f);
    // std::cout<<"RootTemp: "<< root<<std::endl;
    if (root.imag() != 0) { return root; }
  }

  return root;
}

// returns the largest real root to ax^4 + bx^3 + cx^2 + dx + e = 0
float greatest_real_root(float a, float b, float c, float d, float e) {
  // Written with inspiration from: https://en.wikipedia.org/wiki/Quartic_function#General_formula_for_roots
  // std::cout<< "a= " << a<< ";  b= " << b<< ";  c= " << c<< ";  d= " << d<< ";  e= " << e<<";"<<std::endl;

  // Reduce to depressed Quadratic
  float p = (8*a*c-3*b*b)/(8*a*a);
  float q = (b*b*b-4*a*b*c+8*a*a*d)/(8*a*a*a);
  float r = (-3*b*b*b*b+256*a*a*a*e-64*a*a*b*d+16*a*b*b*c)/(256*a*a*a*a);

  // std::cout<<"p = "<<p<<";"<<std::endl;
  // std::cout<<"q = "<<q<<";"<<std::endl;
  // std::cout<<"r = "<<r<<";"<<std::endl;

  // Ferrari's Solution for Quartics: 8m^3 + 8pm^2 + (2p^2-8r)m - q^2 = 0
  complex<float> m = get_cubic_root(8, 8*p, 2*p*p-8*r, -q*q);

  complex<float> root1 = -b/(4*a) + ( sqrt(2.f*m) + sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root2 = -b/(4*a) + ( sqrt(2.f*m) - sqrt(-(2*p + 2.f*m + sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root3 = -b/(4*a) + (-sqrt(2.f*m) + sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;
  complex<float> root4 = -b/(4*a) + (-sqrt(2.f*m) - sqrt(-(2*p + 2.f*m - sqrt(2.f)*q/sqrt(m))))/2.f;

  vector<complex<float>> roots { root1, root2, root3, root4 };

  float max_real_root = 0.f;

  for (complex<float> root: roots) {
    if(root.imag()==0){
    max_real_root = max(max_real_root, root.real());
  }}
  //std::cout<<"Max real root:" << max_real_root<<std::endl;

  return max_real_root;
}

double eigen_real_root_finder(Eigen::VectorXd& coefficient){
  Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
  solver.compute(coefficient);
  const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &root = solver.roots();


  double real_root = 0;
  for(int i = 1;i < root.size();i++){
    if(root[i].imag() == 0){
      real_root = min(real_root, root[i].real());
    }
  }

  for(int i = 1;i < root.size();i++){
    if(root[i].imag() == 0){
      real_root = max(real_root, root[i].real());
    }
  }
  
  return real_root;
}

Eigen::Vector3d convert_depressed_quartic(Eigen::VectorXd& x_coeff){
  Eigen::Vector3d u_coeff;
  double a,b,c,d,e;
  x_coeff(4) = a; x_coeff(3) = b; x_coeff(2) = c; x_coeff(1) = d; x_coeff(0) = e; 
  u_coeff(2) = -3*b*b/(8*a*a) + c/a;
  u_coeff(1) = b*b*b/(8*a*a*a) - b*c/(2*a*a) + d/a;
  u_coeff(0) = -3*b*b*b*b/256*a*a*a*a + c*b*b/(16*a*a*a) - b*d/(4*a*a) + e/a;

  return u_coeff;
}

void updateTransform(vector<Correspondence>& corresponds, Transform& curr_trans) {
  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // You can use the helper functions which are defined above for finding roots and transforming points as and when needed.
  // use helper functions and structs in transform.h and correspond.h
  // input
  //  corresponds : a struct vector of Correspondene struct object defined in correspond.
  //  curr_trans  : A Transform object refernece
  // output(no return)
  //  curr_trans  : update the curr_trans object. Being a call by reference function, 
  //                Any changes you make to curr_trans will be reflected in the calling function in the scan_match.cpp program/

// You can change the number of iterations here. 
// More the number of iterations, slower will be the convergence but more accurate will be the results. 
// You need to find the right balance.
int number_iter = 1;

for(int k = 0; k < number_iter; k++){
  // Fill in the values of the matrics
  Eigen::MatrixXf M_i(2, 4);
  Eigen::Matrix2f C_i;
  Eigen::Vector2f pi_i;

  // Fill in the values for the matrices
  Eigen::Matrix4f M, W;
  Eigen::MatrixXf g(4, 1);
  M << 0, 0, 0, 0, 
       0, 0, 0, 0, 
       0, 0, 0, 0, 
       0, 0, 0, 0;
  W << 0, 0, 0, 0, 
       0, 0, 0, 0, 
       0, 0, 1, 0, 
       0, 0, 0, 1;
  g << 0, 0, 0, 0;

  // M = sum_i M_i^T * C_i * M_i
  // g = sum_i (- 2.0 * M_i^T * C_i^T * pi_i)
  for(int i = 0; i < int(corresponds.size()); i++){
    M_i << 1, 0, corresponds[i].po->getX(), -1.0 * corresponds[i].po->getY(),
           0, 1, corresponds[i].po->getY(), corresponds[i].po->getX();

    C_i = corresponds[i].getNormalNorm() * corresponds[i].getNormalNorm().transpose();
    pi_i = corresponds[i].getPiVec();

    M = M + M_i.transpose() * C_i * M_i;
    g = g - (2.0 * M_i.transpose() * C_i.transpose() * pi_i);
    
  }
  // Check M ang g
  // cout << "Matrix M:\n" << M << "\nMatrix g:\n" << g.transpose() << endl;

  // Define sub-matrices A, B, D from M
  Eigen::Matrix2f A, B, D;
  A = 2.0 * M.topLeftCorner(2,2);
  B = 2.0 * M.topRightCorner(2,2);
  D = 2.0 * M.bottomRightCorner(2,2);

  // Define S and S_A matrices from the matrices A B and D
  Eigen::Matrix2f S, S_A;
  S   = D - B.transpose() * A.inverse() * B;
  S_A = S.determinant() * S.inverse();

  // Check S 
  // cout << "Matrix S:\n" << S << endl;

  // Find the coefficients of the quadratic function of lambda
  Eigen::Vector3f pow;
  Eigen::Matrix4f pow_22, pow_11, pow_00;
  pow_22 << A.inverse()*B*B.transpose()*A.inverse().transpose(), -1.0*A.inverse()*B,
            -1.0*B.transpose()*A.inverse().transpose()         , Eigen::MatrixXf::Identity(2,2);
  pow_11 << A.inverse()*B*S_A*B.transpose()*A.inverse().transpose(), -1.0*A.inverse()*B*S_A,
            -1.0*S_A*B.transpose()*A.inverse().transpose()         , S_A;
  pow_00 << A.inverse()*B*S_A*S_A.transpose()*B.transpose()*A.inverse().transpose(), -1.0*A.inverse()*B*S_A*S_A.transpose(),
            -1.0*S_A*S_A.transpose()*B.transpose()*A.inverse().transpose()         , S_A*S_A.transpose();

  pow << 4.0 * g.transpose() * pow_22 * g, 
         4.0 * g.transpose() * pow_11 * g, 
         1.0 * g.transpose() * pow_00 * g;
  

  // Find the value of lambda by solving the equation formed. You can use the greatest real root function
  double lambda;
  Eigen::VectorXd coeff(5);
  coeff(4) = 16;
  coeff(3) = 16 * (S(0,0) + S(1,1));
  coeff(2) = 4 * (2*S.determinant() + (S(0,0) + S(1,1))*(S(0,0) + S(1,1))) - pow(2);
  coeff(1) = 4 * S.determinant() * (S(0,0) + S(1,1)) - pow(1);
  coeff(0) = S.determinant() * S.determinant() - pow(0);

  // lambda = greatest_real_root(coeff(4), coeff(3), coeff(2), coeff(1), coeff(0));
  lambda = eigen_real_root_finder(coeff);

  // Print coeff and lambda
  // cout << "coeff:\t" << coeff.transpose().reverse() << endl;
  // cout << "Lambda:\t" << lambda << endl;

  // Find the value of x which is the vector for translation and rotation
  Eigen::Vector4f x;
  Eigen::Matrix4f temp44;
  temp44 = 2.0*(M + lambda * W);
  x = -1.0 * temp44.inverse().transpose() * g;

  // Check x
  // cout << "Vector x:\t" << x.transpose() << endl;

  // Convert from x to new transform  
  float theta = atan2(x(3), x(2));
  curr_trans = Transform(x(0), x(1), theta); 

  // cout << "curr_trans\n\tX:\t" << curr_trans.x_disp << endl;
  // cout << "\tY:\t" << curr_trans.y_disp << endl;
  // cout << "\tTheta:\t" << curr_trans.theta_rot << endl << endl;
}
}