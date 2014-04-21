#include <iostream>
#include <vector>

#include <Debug.hh>
#include <Mathematics/Bsplines.hh>


using namespace::std;
using namespace::PatternGeneratorJRL;



Bsplines::Bsplines(int degree, std::vector<Point> control_points)
{
    m_degree = degree;

    m_control_points.clear();
    m_control_points = control_points;
    m_knot_vector.clear();
    GenerateKnotVector("centripetal");
}

Bsplines::~Bsplines()
{
}

void Bsplines::GenerateKnotVector(std::string method)
{
    /*Calculer set of parameters*/
    int i,j;
    vector<double> set_of_pam;

        if (method == "centripetal")
        {
            cout << "centripetal" << endl;
            set_of_pam.clear();
            set_of_pam.reserve(m_control_points.size());
            cout << m_control_points.size() << endl;
            double L = 0.0;
            double D = 0.0;
            for (i=0;i<m_control_points.size()-1;i++)
            {
                L += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
            }
            for (i=0;i<m_control_points.size();i++)
            {
                if (i == 0)
                {
                    set_of_pam[0] = 0.0;
                }
                else if (i == m_control_points.size()-1)
                {
                    set_of_pam[i] = 1;
                }
                else
                {
                    D += sqrt ( sqrt(((m_control_points[i].x - m_control_points[i+1].x)*(m_control_points[i].x - m_control_points[i+1].x)) +
                              ((m_control_points[i].y - m_control_points[i+1].y)*(m_control_points[i].y - m_control_points[i+1].y)) ) );
                    set_of_pam[i] = D/L;
                }
            }
            /*generate a knot vector*/
            //cout << m_control_points.size() << endl;

            m_knot_vector.clear();
            double U = 0.0;
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(0.0);
            }

            if (m_control_points.size()-1>=m_degree)
            {
                for (i=1;i<=m_control_points.size()-1-m_degree;i++)
                {
                    j=0;
                    U=0.0;
                    while (j<m_degree)
                    {
                        U +=set_of_pam[i];
                        j++;
                    }
                    m_knot_vector.push_back((U/m_degree));
                }
            }
            //cout << m_control_points.size() << endl;

            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(1.0);
            }
            if (m_knot_vector.size() - 1 != m_control_points.size() + m_degree)
            {
                cout << "Knot vector cant be created. m_control_points.size()-1>=m_degree "<< endl;
                m_knot_vector.clear();
            }
        }

        else if (method  == "universal")
        {
            m_knot_vector.clear();
            cout << "universal" << endl;
            double U;
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(0.0);
            }
            for (i=1;i<=m_control_points.size()-1-m_degree;i++)
            {
                U = double(i)/(m_control_points.size()-1-m_degree+1);
                m_knot_vector.push_back(U);
            }
            for (i=0;i<=m_degree;i++)
            {
                m_knot_vector.push_back(1.0);
            }
        }

}

double *Bsplines::ComputeBasisFunction(double t)
{
    double **m_basis_function;
    m_basis_function = new double* [m_degree+1];
    int i,j;

    for (j=0; j<m_degree+1; j++)
        {
            int n = m_knot_vector.size() - 1 -j -1;
            m_basis_function[j] = new double[n+1];
        }
        for(j=0;j <= m_degree;j++)
        {
            int n = m_knot_vector.size() - 1 -j -1;
            cout << "order  " ; cout << j << endl;
            cout << "n control point  " ; cout << n << endl;

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
                    if ( m_knot_vector[i] != m_knot_vector[i+j] && m_knot_vector[i+j+1] != m_knot_vector[i+1] )
                        m_basis_function[j][i]= m_basis_function[j-1][i]*(t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i])
                                                + m_basis_function[j-1][i+1]*(m_knot_vector[i+j+1] - t)/ (m_knot_vector[i+j+1]-m_knot_vector[i+1] );

                    else if ( m_knot_vector[i] == m_knot_vector[i+j] && m_knot_vector[i+j+1] == m_knot_vector[i+1] )
                        m_basis_function[j][i] = 0.0;

                    else if (m_knot_vector[i] == m_knot_vector[i+j])
                        m_basis_function[j][i]= m_basis_function[j-1][i+1]*(m_knot_vector[i+j+1] - t)/ (m_knot_vector[i+j+1]-m_knot_vector[i+1] );

                    else if (m_knot_vector[i+j+1] == m_knot_vector[i+1])
                        m_basis_function[j][i]= m_basis_function[j-1][i]*(t - m_knot_vector[i])/(m_knot_vector[i+j]-m_knot_vector[i]);
                }
            }
        }

return m_basis_function[m_degree];
}

Point Bsplines::ComputeBplines(double t)
{
    double *m_basis_function = ComputeBasisFunction(t);

    Point C = {0.0,0.0};
    if (m_degree == ((m_knot_vector.size()-1) - (m_control_points.size()-1)- 1) )
        {
            for (int i=0;i<m_control_points.size();i++)
            {
                C.x += m_basis_function[i] * m_control_points[i].x;
                C.y += m_basis_function[i] * m_control_points[i].y;
            }
        }
return C;
}

Bsplines Bsplines::DerivativeBsplines()
{
    std::vector<Point> Q;
    Q.clear();
    Q.reserve(m_control_points.size()-1);

    Point T;
    if (m_degree >=1)
    {
        for (int i=0;i<m_control_points.size()-1;i++)
        {

            //cout << i<< endl;
            T.x = ((m_control_points[i+1].x - m_control_points[i].x)*double(m_degree) )/ (m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]);
            //cout << i<< endl;
            T.y = ((m_control_points[i+1].y - m_control_points[i].y)*double(m_degree) )/ (m_knot_vector[i+m_degree+1] - m_knot_vector[i+1]);
            //cout << i<< endl;
            Q.push_back(T);
            //cout << i << endl;
        }
        Bsplines B(m_degree-1,Q);
        std::vector<double> new_knot_vector(m_knot_vector.begin()+1,m_knot_vector.end()-1);
        B.SetKnotVector(new_knot_vector);
        cout <<"derivative function "<<endl;
        B.PrintKnotVector();
        B.PrintControlPoints();
        B.PrintDegree();
        return B;
        }
    else
        {
            std::cout << "the function cannot be derivative " << std::endl;
            return Bsplines(m_degree,m_control_points);
        }

}

void Bsplines::SetDegree(int &degree)
{
    m_degree = degree;
}

void Bsplines::SetControlPoints(std::vector<Point> &control_points)
{
    if (control_points.size()>=2)
    {
        m_control_points = control_points;
    }
    else
    {
        std::cout << "You must give at least 2 control points" << std::endl;
    }
}

void Bsplines::SetKnotVector(std::vector<double> &knot_vector)
{
    m_knot_vector = knot_vector;
}

int Bsplines::GetDegree() const
{
    return m_degree;
}

std::vector<Point> Bsplines::GetControlPoints() const
{
    return m_control_points;
}

std::vector<double> Bsplines::GetKnotVector() const
{
    return m_knot_vector;
}

void Bsplines::PrintKnotVector() const
{
    std::cout << "Knot Vector: "<< std::endl;
    for (int i = 0;i<m_knot_vector.size();i++)
    {
        std::cout << m_knot_vector[i] << " , ";
    }
    std::cout <<" " <<std::endl;
}

void Bsplines::PrintControlPoints() const
{
    std::cout << "Control Points : "<< std::endl;
    for (int i = 0;i<m_control_points.size();i++)
    {
        std::cout << m_control_points[i].x << " , " << m_control_points[i].y << std::endl;
    }
}

void Bsplines::PrintDegree() const
{
    std::cout << "Degree: " << m_degree << std::endl;
}
