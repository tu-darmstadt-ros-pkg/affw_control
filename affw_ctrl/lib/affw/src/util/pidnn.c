/*
 * pidnn.c
 *
 * http://molefrog.com/pidnn-talk/#slide-12
 *
 *  Created on: Jun 9, 2016
 *      Author: Nicolai Ommer <nicolai.ommer@gmail.com>
 */

typedef double d_type;

d_type w_out[3];
d_type w_in[2][3];

d_type p_function(d_type u)
{
	if(u < -1) return -1;
	if(u > 1) return 1;
	return u;
}

d_type i_function(d_type u, d_type x)
{
	if(x < -1) return -1;
	if(x > 1) return 1;
	return x + u;
}

d_type d_function(d_type u, d_type u_p, d_type x)
{
	if(x < -1) return -1;
	if(x > 1) return 1;
	return u - u_p;
}

void train(d_type *r, d_type *y, int n)
{
	double eta = 1;
	for(int i=0;i<3;i++)
	{
		d_type sum = 0;
		for(int j=0;j<n;j++)
		{
			sum += (r[j] - y[j]) * (1);
		}
		d_type dJ = -2.0/n;
		w_out[i] = w_out[i] - eta * dJ;
	}
}

void pidnn()
{

}
