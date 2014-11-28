#include <QtGui>

#include "MainWindow.h"
#include "cholmod_matrix.h"
#include <assert.h>

int main(int argc, char **argv)
{
	cholmod_initialize();


	CholmodSparseMatrix matrix;
	matrix.reshape(3,3,100);

	matrix.startMatrixFill();
	matrix.addElement(0,0,  2.0860712409449427);
	matrix.addElement(0,1, -0.6454601394461954);

	matrix.addElement(1,0, -0.0058869607834046832);
	matrix.addElement(1,1, 1.8891482357926273);
	matrix.addElement(1,2, -0.19544543345678983);

	matrix.addElement(2,1, -0.29527365400366651);
	matrix.addElement(2,2, 2.0248467947898217);

	cholmod_dense* RHS = cholmod_zeros(3, 1, CHOLMOD_REAL, cholmod_get_common());
	double *RHS_X = (double*)RHS->x;

	RHS_X[0] = -0.00076063855109353984;
	RHS_X[1] = -0.0028624150027941608;
	RHS_X[2] = -0.001146138433778976;

	cholmod_sparse cholmod_matrix;
	cholmod_matrix.nrow = matrix.numCols(); // P is row-major, so cholmod thinks it's P*
	cholmod_matrix.ncol = matrix.numRows();
	cholmod_matrix.nzmax = matrix.getNumNonzero();
	cholmod_matrix.p = matrix.getAp();
	cholmod_matrix.i = matrix.getAi();
	cholmod_matrix.x = matrix.getAx();
	cholmod_matrix.z = NULL;
	cholmod_matrix.stype = 0;
	cholmod_matrix.itype = CHOLMOD_INT;
	cholmod_matrix.xtype = CHOLMOD_REAL;
	cholmod_matrix.dtype = CHOLMOD_DOUBLE;
	cholmod_matrix.sorted = 1;
	cholmod_matrix.packed = TRUE;
	cholmod_matrix.nz = NULL;

	cholmod_factor *L = cholmod_analyze(&cholmod_matrix, cholmod_get_common());
	assert (L != NULL);
	cholmod_factorize(&cholmod_matrix, L, cholmod_get_common());
	cholmod_dense * Xcholmod = cholmod_solve(CHOLMOD_A, L, RHS, cholmod_get_common());

	double *result = (double*)Xcholmod->x;

	for (int i = 0 ; i < 3 ; i++)
		printf("%f \n", result[i]);



	QApplication app(argc, argv);

    MainWindow *window = new MainWindow();
    window->show();
    //window->resize(1024, 768);
    app.exec();

    delete window;
    cholmod_finalize();
}
