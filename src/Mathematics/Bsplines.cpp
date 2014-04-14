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

Bsplines::~Bsplines()
{
}

void Bsplines::SetBasisFunctionParameters(int order; std::vector<double> knot_vector)
{
    m_knot_vector = knot_vector;
    m_order = order;
}

double **Bsplines::ComputeBasisFunction(double t)
{
    double **basis_function;
    basis_function = new double* [m_order];
    /* Allocation dynamic */
    for (int j=0; j<=m_order+1; j++)
    {
        int n = m_knot_vector.size() - 1 -j -1;
        basis_function[j] = new double[n];
    }
    /* Calculate the basis function */
    for(j=0;j <= m_order;j++)
    {
        int n = m_knot_vector.size() - 1 -j -1;
        cout << "order  " ; cout << j << endl;
        cout << "n control point  " ; cout << n << endl;

        for(i=0;i <= n;i++)
        {

            if (j == 0 && m_knot_vector[i]<= t && t< m_knot_vector[i+1])
            {
                basis_function[j][i] = 1.0;
            }
            else if (j == 0)
            {
                basis_function[j][i] = 0.0;
            }
            else if (j!=0)
            {
                basis_function[j][i]= basis_function[j-1][i]*(t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i]) + basis_function[j-1][i+1]*(m_knot_vector[i+j+1] - t)/(m_knot_vector[i+j+1]-m_knot_vector[i+1] );
            }
            cout << j  << "  ";
            cout << i << "  ";
            cout << basis_function[j][i] << "  ";
            cout << endl;
        }
        cout << endl;
    }
    /*for (int i=0;i<m_control_points.size(),i++)
    {
        C.x += basic_function[m_order][i] * m_control_points[i].x;
        C.y += basic_function[m_order][i] * m_control_points[i].y;
    }*/
return basis_function;
}

double Bsplines::ComputeBplines(double **basis_function)
{
    double C = 0.0;
    if (m_order == ((m_knot_vector.size()-1) - (m_control_points.size()-1)- 1)
        {
            for (int i=0;i<m_control_points.size(),i++)
            {
                C.x += basic_function[m_order][i] * m_control_points[i].x;
                C.y += basic_function[m_order][i] * m_control_points[i].y;
            }
        }
return C;
}

void Bsplines::SetOrder(int order)
{
    m_order = order;
}

void Bsplines::SetControlPoints(std::vector<double> control_points)
{
    m_control_points = control_points;
}

void Bsplines::SetKnotVector(std::vector<double> knot_vector)
{
    m_knot_vector = knot_vector;
}

int Bsplines::GetOrder() const
{
    return m_order;
}

std::vector<Point> Bsplines::GetControlPoints() const
{
    return m_control_points;
}

std::vector<double> Bsplines::GetKnotVector() const
{
    return m_knot_vector;
}


