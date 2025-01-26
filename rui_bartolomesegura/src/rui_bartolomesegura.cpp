#include <Eigen/Dense>
#include <iostream>


int main(int argc, char **argv) {
    // Mine JMBAG
    int JMAB[10] = {0, 0, 3, 6, 5, 7, 2, 6, 7, 6};

    // Define matrix
    Eigen::MatrixXd m1(5,5);
    Eigen::MatrixXd m2(5,5);

    // Define identity
    Eigen::MatrixXd identity(5,5);
    identity.setIdentity();

    // Define vector
    Eigen::VectorXd v(5);

    // Values of m1
    for (int y = 0; y < 5; y++){
        m1(0,y) = JMAB[y];
        m1(1,y) = JMAB[y];
        m1(2,y) = JMAB[y];
        m1(3,y) = JMAB[y];
        m1(4,y) = JMAB[y];
    }

    // Values of m2
    m2 = m1 + identity;

    // Values of v.
    for (int x = 0; x < 5; x++){
        // +5 To obtain the especified value of the JMBAG
        v(x) = JMAB[x + 5];
    }

    std::cout << "m2 * v:\n" << m2 * v << std::endl;

    std::cout << "\nv * v transpose:\n" << v * v.transpose() << "\n" << std::endl;

    std::cout << "\nm1 + m2:\n" << m1 + m2 << "\n" << std::endl;

    std::cout << "\nm2 inverse:\n" << m2.inverse() << "\n" << std::endl;

    std::cout << "\nm2 transpose:\n" << m2.transpose() << "\n" << std::endl;

    return 0;
}
