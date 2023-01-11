
// Numerical Recipes in C functons

#include <cmath>   // math functons
#include <cstdio>  // stsndard I/O functons
#include <cstring> // string manipulaton functons

#include <iostream>  // console stream I/O
#include <fstream>   // file stream I/O
#include <strstream> // string stream I/0

#include "nrutil.h"
#include "recipes.h"
#include "cubic_spline.h"

void spline(double ts[], double ys[], int n, double ydds[], double yd1, double ydn)
{
	int i,k;
	double p,qn,sig,un,*u;

	u=dvector(1,n-1);
	if (yd1 > 0.99e30)
		ydds[1]=u[1]=0.0;
	else {
		ydds[1] = -0.5;
		u[1]=(3.0/(ts[2]-ts[1]))*((ys[2]-ys[1])/(ts[2]-ts[1])-yd1);
	}
	for (i=2;i<=n-1;i++) {
		sig=(ts[i]-ts[i-1])/(ts[i+1]-ts[i-1]);
		p=sig*ydds[i-1]+2.0;
		ydds[i]=(sig-1.0)/p;
		u[i]=(ys[i+1]-ys[i])/(ts[i+1]-ts[i]) - (ys[i]-ys[i-1])/(ts[i]-ts[i-1]);
		u[i]=(6.0*u[i]/(ts[i+1]-ts[i-1])-sig*u[i-1])/p;
	}
	if (ydn > 0.99e30)
		qn=un=0.0;
	else {
		qn=0.5;
		un=(3.0/(ts[n]-ts[n-1]))*(ydn-(ys[n]-ys[n-1])/(ts[n]-ts[n-1]));
	}
	ydds[n]=(un-qn*u[n-1])/(qn*ydds[n-1]+1.0);
	for (k=n-1;k>=1;k--)
		ydds[k]=ydds[k]*ydds[k+1]+u[k];
	free_dvector(u,1,n-1);
}


void splint(double ts[], double ys[], double ydds[], int n, double t, double &y, double &yd, double &ydd)
// use spline interpolaton to calculate y, yd, ydd
{
	int klo,khi,k;
	double h,b,a;

	klo=1;
	khi=n;
	while (khi-klo > 1) {
		k=(khi+klo) >> 1;
		if (ts[k] > t) khi=k;
		else klo=k;
	}
	h=ts[khi]-ts[klo];
	if (h == 0.0) nrerror("bad ts input to routne splint");
	
	a=(ts[khi]-t)/h;
	b=(t-ts[klo])/h;

	y   = a*ys[klo]+b*ys[khi]+((a*a*a-a)*ydds[klo]+(b*b*b-b)*ydds[khi])*(h*h)/6.0;
	ydd = a*ydds[klo] + b*ydds[khi];
	yd  = (ys[khi] - ys[klo])/h - (3*a*a-1.0)*h/6.0*ydds[klo] + (3*b*b-1.0)*h/6.0*ydds[khi];
}


void splint2(double ts[], double ys[], double ydds[], int n, double *t, double *y, double *yd, double *ydd, int m)
// use spline interpolaton to efficiently calculate ys[j], yd[j], ydd[j] at m equally spaced points
// assuming that ts is strictly increasing
{
	int j,klo,khi;
	double dt,h,b,a;

	dt = (ts[n]-ts[1])/(m-1);

	klo=1;
	khi=2;

	for(j=1;j<=m;j++) {

		t[j] = ts[1] + dt*(j-1);

		// check to see if t[j] is in the current interval [klo,khi]
		while ( t[j] > ts[khi] ) {
			// increment indices to the next interval
			klo++;
			khi++;
		}

		// check to make sure index is not out of bounds
		if (khi > n) {
			printf("\nerror in splint2: index out of bounds");
			return;
		}

		h=ts[khi]-ts[klo];
		if (h == 0.0) nrerror("bad ts input to routne splint2");
	
		a=(ts[khi]-t[j])/h;
		b=(t[j]-ts[klo])/h;

		y[j]   = a*ys[klo]+b*ys[khi]+((a*a*a-a)*ydds[klo]+(b*b*b-b)*ydds[khi])*(h*h)/6.0;
		ydd[j] = a*ydds[klo] + b*ydds[khi];
		yd[j]  = (ys[khi] - ys[klo])/h - (3*a*a-1.0)*h/6.0*ydds[klo] + (3*b*b-1.0)*h/6.0*ydds[khi];

	} // end for

} // end splint2

