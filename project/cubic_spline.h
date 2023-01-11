
// Numerical Recipes in C functons

// calculate a cubic spline
void spline(double ts[], double ys[], int n, double ydds[], double yd1 = 1.0e35, double ydn = 1.0e35);

// use spline interpolaton to calculate y, yd, ydd 
void splint(double ts[], double ys[], double ydds[], int n, double t, double &y, double &yd, double &ydd);

// use spline interpolaton to calculate t[j], y[j], yd[j], ydd[j] at m equally spaced points
// assuming that ts is strictly increasing
void splint2(double ts[], double ys[], double ydds[], int n, double *t, double *y, double *yd, double *ydd, int m);
