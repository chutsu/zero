#include <Eigen/Dense>

#include "zero/math.h"

int main() {
	for (size_t k = 1; k < 1000; k++) {
		size_t m = k;
		Eigen::MatrixXd A;
		Eigen::MatrixXd B;
		Eigen::MatrixXd C;
    A.resize(m, m);
    B.resize(m, m);
    C.resize(m, m);

		for (size_t i = 0; i < m; i++) {
			for (size_t j = 0; j < m; j++) {
				A(i, j) = randf(-1.0, 1.0);
				B(i, j) = randf(-1.0, 1.0);
			}
		}

		struct timespec t = tic();
		C = A * B;
    printf("Matrix size [%ld] Finnished in %f seconds. \n", m, toc(&t));
	}

  return 0;
}
