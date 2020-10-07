#include<Eigen/Eigen>
#include <iostream>
#include <math.h>

using namespace Eigen;

double position(VectorXd a, double t) {
    VectorXd t_vec(6);
    t_vec << 1, t, pow(t,2), pow(t,3), pow(t,4), pow(t,5);
    double q_t = t_vec.dot(a);
    return q_t;
}

VectorXd calculate_a(double t_0, double t_f, VectorXd b) {
    MatrixXd qp(6,6);
    qp << 1, t_0, pow(t_0, 2), pow(t_0, 3), pow(t_0, 4), pow(t_0, 5),
        0, 1, 2*t_0, 3*pow(t_0, 2), 4*pow(t_0, 3), 5*pow(t_0, 4),
        0, 0, 2, 6*t_0, 12*pow(t_0, 2), 20*pow(t_0, 3),
        1, t_f, pow(t_f, 2), pow(t_f, 3), pow(t_f, 4), pow(t_f, 5),
        0, 1, 2*t_f, 3*pow(t_f, 2), 4*pow(t_f, 3), 5*pow(t_f, 4),
        0, 0, 2, 6*t_f, 12*pow(t_f, 2), 20*pow(t_f, 3);

    MatrixXd qp_inv(6,6);
    qp_inv << qp.inverse();
    VectorXd a(6);
    a << qp_inv*b;
    return a;

}

int main(int argc, char *argv[]){
    std::cout << "Hello quintic" << "\n";

    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << "Here is the matrix m:\n" << m << std::endl;
    VectorXd v(2);
    v(0) = 4;
    v(1) = v(0) - 1;
    std::cout << "Here is the vector v:\n" << v << std::endl;

    int num = 1;

    Matrix3f mat;
    mat << num, 2, 3,
        4, num, 6,
        7, 8, num;
    std::cout << mat << "\n";

    std::cout << "Multiplication \n" << m*v << "\n";

    std::cout << "Inverse \n" << m.inverse() << "\n";


    MatrixXd qp(6,6);
    double t_0 = 0.0;
    double t_f = 2.0;
    double q_0 = 0.0;
    double q_f = 0.3;
    double v_0 = 0.0; 
    double v_f = 0.0;
    double alpha_0 = 0.0;
    double alpha_f = 0.0;

    qp << 1, t_0, pow(t_0, 2), pow(t_0, 3), pow(t_0, 4), pow(t_0, 5),
        0, 1, 2*t_0, 3*pow(t_0, 2), 4*pow(t_0, 3), 5*pow(t_0, 4),
        0, 0, 2, 6*t_0, 12*pow(t_0, 2), 20*pow(t_0, 3),
        1, t_f, pow(t_f, 2), pow(t_f, 3), pow(t_f, 4), pow(t_f, 5),
        0, 1, 2*t_f, 3*pow(t_f, 2), 4*pow(t_f, 3), 5*pow(t_f, 4),
        0, 0, 2, 6*t_f, 12*pow(t_f, 2), 20*pow(t_f, 3);

    MatrixXd qp_inv(6,6);
    qp_inv << qp.inverse();

    VectorXd b(6);

    b << q_0, v_0, alpha_0, q_f, v_f, alpha_f;

    VectorXd a(6);
    a << qp_inv*b;



    std::cout << "Qp \n" << qp << "\n";

    std::cout << "Qp inv \n" << qp.inverse() << "\n";

    std::cout << "Mul \n" << qp*qp.inverse() << "\n";

    std::cout << "a \n" << a << "\n";

    for (int t = 0; t < 1000; t++) {
        double x = position(a,t*(t_f/1000));
        std::cout << "x: " << x << "\n";
    }

    VectorXd b_init(6);
    VectorXd b_mid(6);
    VectorXd b_end(6);

    VectorXd a_init(6);
    VectorXd a_mid(6);
    VectorXd a_end(6);

    b_init << 0.0, 0.0, 0.0, 0.01, 0.1, 0.0;
    b_mid << 0.0, 0.1, 0.0, 0.01, 0.1, 0.0;
    b_end << 0.0, 0.1, 0.0, 0.01, 0.0, 0.0;

    a_init = calculate_a(0.0, 0.04, b_init);
    a_mid = calculate_a(0.0, 0.04, b_mid);
    a_end = calculate_a(0.0, 0.04, b_end); 


    std::cout << "a inint from func \n" << a_init << "\n";
    std::cout << "a m from func \n" << a_mid << "\n";
    std::cout << "a e from func \n" << a_end << "\n";
    




}