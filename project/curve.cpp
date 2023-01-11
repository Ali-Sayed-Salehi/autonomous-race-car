
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "nrutil.h"
#include "recipes.h"
#include "cubic_spline.h"

#include "curve.h"

using namespace std;

// file name of curve
char curve_file_name[] = "spline_fit.txt";


int curve(double t, double &x, double &y, double &xd, double &yd, 
			double &xdd, double &ydd)
{
	int k, kmax;	
	ifstream fin;
	static double ts_min, ts_max;
	static int init = 0, nc;
	static double *ts, *xs, *xdds, *ys, *ydds;

	// initialization section -- only executed once
	if( !init ) {
		
		kmax = 100000; // maximum assumed array / file size
		
		// allocate arrays		
		ts  = dvector(1,kmax);
		xs  = dvector(1,kmax);
		xdds = dvector(1,kmax);	
		ys  = dvector(1,kmax);
		ydds = dvector(1,kmax);
		
		fin.open(curve_file_name);
		
		ts_min = 1.0e10;
		ts_max = -1.0e10;
		
		k = 1;	
		while(k < kmax) {
			
			fin >> ts[k];
			
			// putting break here prevents against blank last line
			if( fin.eof() ) break; 
			
			fin >> xs[k];
			fin >> ys[k];

			if( ts[k] > ts_max ) ts_max = ts[k];
			if( ts[k] < ts_min ) ts_min = ts[k];

			k++;
		}		
	
		// number of points
		nc = k-1;

		fin.close();
		
		// compute cubic spline coefficients
		spline(ts,xs,nc,xdds); // x	
		spline(ts,ys,nc,ydds); // y
		
		init = 1;
	}

	// cubic spline interpolation
	splint(ts,xs,xdds,nc,t,x,xd,xdd); // x
	splint(ts,ys,ydds,nc,t,y,yd,ydd); // y
	
	// subtract offset to put the origin at the center of track
	x = x - 7595.0/2;
	y = y - 2545.0/2;
	
	return 0;
}

