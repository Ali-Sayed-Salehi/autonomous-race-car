
// Numerical Recipes in C functions

#include <cmath>   // math functions
#include <cstdio>  // standard I/O functions
#include <cstring> // string manipulation functions

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include "recipes.h"

#include "nrutil.h"

// numerical recipes utility functions

float *vector(int nl, int nh)
{
	float *v;

	v=(float *)malloc((unsigned) (nh-nl+1)*sizeof(float));
	if (!v) nrerror("allocation failure in dvector()");
	
	// initialize vector to zero
	for(int i = 0; i<(nh-nl+1); i++) v[i] = 0.0;
	
	return v-nl;
}

double *dvector(int nl, int nh)
{
	double *v;

	v=(double *)malloc((unsigned) (nh-nl+1)*sizeof(double));
	if (!v) nrerror("allocation failure in dvector()");
	
	// initialize dvector to zero
	for(int i = 0; i<(nh-nl+1); i++) v[i] = 0.0;
	
	return v-nl;
}

int *ivector(int nl, int nh)
{
	int *v;

	v=(int *)malloc((unsigned) (nh-nl+1)*sizeof(int));
	if (!v) nrerror("allocation failure in ivector()");
	
	// initialize ivector to zero
	for(int i = 0; i<(nh-nl+1); i++) v[i] = 0;
	
	return v-nl;
}

double **dmatrix(int nrl, int nrh, int ncl, int nch)
{
	int i;
	double **m;

	m=(double **) malloc((unsigned) (nrh-nrl+1)*sizeof(double*));
	if (!m) nrerror("allocation failure 1 in dmatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(double *) malloc((unsigned) (nch-ncl+1)*sizeof(double));
		if (!m[i]) nrerror("allocation failure 2 in dmatrix()");
		m[i] -= ncl;
	}
	
	// initialize dmatrix to zero
	for(int i = nrl;i <= nrh;i++)
		for(int j = ncl;j <= nch;j++)
			m[i][j] = 0.0;
	
	return m;
}


float **matrix(int nrl, int nrh, int ncl, int nch)
{
	int i;
	float **m;

	m=(float **) malloc((unsigned) (nrh-nrl+1)*sizeof(float*));
	if (!m) nrerror("allocation failure 1 in matrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(float *) malloc((unsigned) (nch-ncl+1)*sizeof(float));
		if (!m[i]) nrerror("allocation failure 2 in matrix()");
		m[i] -= ncl;
	}
	
	// initialize matrix to zero
	for(int i = nrl;i <= nrh;i++)
		for(int j = ncl;j <= nch;j++)
			m[i][j] = 0.0;	
	
	return m;
}


void free_vector(float *v, int nl, int nh)
{
	free((char*) (v+nl));
}

void free_dvector(double *v, int nl, int nh)
{
	free((char*) (v+nl));
}

void free_ivector(int *v, int nl, int nh)
{
	free((char*) (v+nl));
}

void free_dmatrix(double **m, int nrl, int nrh, int ncl, int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

void free_matrix(float **m, int nrl, int nrh, int ncl, int nch)
{
	int i;

	for(i=nrh;i>=nrl;i--) free((char*) (m[i]+ncl));
	free((char*) (m+nrl));
}

