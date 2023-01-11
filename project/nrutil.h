
// numerical recipes utility functions

float *vector(int nl, int nh);

double *dvector(int nl, int nh);

int *ivector(int nl, int nh);

double **dmatrix(int nrl, int nrh, int ncl, int nch);

float **matrix(int nrl, int nrh, int ncl, int nch);

void free_vector(float *v, int nl, int nh);

void free_dvector(double *v, int nl, int nh);

void free_ivector(int *v, int nl, int nh);

void free_dmatrix(double **m, int nrl, int nrh, int ncl, int nch);

void free_matrix(float **m, int nrl, int nrh, int ncl, int nch);

