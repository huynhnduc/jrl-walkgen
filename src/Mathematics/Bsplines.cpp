#include <iostream>
#include <vector>

#include <jrl/mal/matrixabstractlayer.hh>

#include <Mathematics/Bsplines.hh>

#include <Debug.hh>

using namespace::std;
using namespace::PatternGeneratorJRL;



Bsplines::Bsplines()
{
    m_order = 0;
    m_control_points.clear();
    m_knot_vector.clear();
}

void Bsplines::SetBsplinesParameters(std::vector<Point> control_points,
                                     std::vector<double> knot_vector)
{
    m_control_points = control_points;
    m_knot_vector = knot_vector;
    m_order = ( m_knot_vector.size() -1 ) - ( m_control_point.size() - 1 ) - 1;
}

Bsplines::~Bsplines()
{
}

void Bsplines::ComputeBasisFunction(double t)
{
    double **m_basis_function;
    m_basis_function = new double* [m_order];
    /* Allocation dynamic */
    for (int j=0; j<=m_order+1; j++)
    {
        //cout << j << endl;
        int n = m_knot_vector.size() - 1 -j -1;
        m_basis_function[j] = new double[n];
    }
    /*Calculate the basis function*/
    for(j=0;j <= m_order;j++)
    {
        int n = m_knot_vector.size() - 1 -j -1;
        cout << "order  " ; cout << j << endl;
        cout << "n control point  " ; cout << n << endl;
        //m_basis_function[j] = new double[n];
        for(i=0;i <= n;i++)
        {

            if (j == 0 && m_knot_vector[i]<= t && t< m_knot_vector[i+1])
            {
                m_basis_function[j][i] = 1.0;
            }
            else if (j == 0)
            {
                m_basis_function[j][i] = 0.0;
            }
            else if (j!=0)
            {
                m_basis_function[j][i]= m_basis_function[j-1][i]*(t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i]) + m_basis_function[j-1][i+1]*(m_knot_vector[i+j+1] - t)/ (m_knot_vector[i+j+1]-m_knot_vector[i+1] );
            }
            cout << j  << "  ";
            cout << i << "  ";
            cout << m_basis_function[j][i] << "  ";
            cout << endl;
        }
        cout << endl;
      //  m_basis_function[i+1][j] = 0.0;
    }        //m_basis_function[i+1][j] = 0.0;

}

int Bsplines::GetOrder() const
{
    return m_order;
}

std::vector<Point> Bsplines::GetControlPoint() const
{
    return m_control_points;
}

std::vector<double> Bsplines::GetKnotVector() const
{
    return m_knot_vector;
}


