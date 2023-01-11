
// header file for Numerical Recipes functions

// note that all matrices and vectors must be 1 offset ie. a[1..n][1..n]

void nrerror(char *c);
// error message function

void gaussj(double **a, int n, double **b, int m);
/*
Gauss Jordan elimination to solve Ax=b
INPUT:
a - nxn matrix A
b - nxm matrix that contains m b-vectors
OUTPUT:
a - inverse of matrix
b - solution vectors
*/

void ludcmp(double **a, int n, int *indx, double *d);
/*
LU decomposition of a matrix
INPUT:
a - nxn matrix A
OUTPUT:
a - LU decomposition of INPUT a
indx - row index
*/

void lubksb(double **a, int n, int *indx, double *b);
/* Solve Ax=b using the LU decomposition of a matrix.
INPUT:
a - LU decomposition of nxn matrix A
indx - row index
b - right hand vector
OUTPUT:
b - the solution vector x
*/

void mprove(double **a, double **alud, int n, int *indx, double *b,
            double *x);
/*
Iterative improvement of the solution of Ax=b using the LU decomposition
of a matrix.
INPUT:
alud - LU decomposition of nxn matrix A
indx - row index 
OUTPUT:
x - improved solution vector x
*/

void svdcmp(double **a, int m, int n, double *w, double **v);
/* Calculate the Singular Value Decomposition of a matrix A = UWV'.
// INPUT:
a - the mxn matrix A
// OUTPUT:
a - the U matrix
w - the diagonal values of the diagonal matrix W
v - the nxn V matrix
*/

void svbksb(double **u, double *w, double **v, int m, int n, double *b,
            double *x);
/*
Solve Ax=b using the SVD of a matrix.
INPUT:
a - the mxn matrix A
u - the mxn matrix U
v - the nxn matrix V
w - the n vector W
OUTPUT:
x - the n component solution vector
*/