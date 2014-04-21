
/** \file Bsplines.h
    \brief Bsplines object for generating trajectoire from set of Points given. */


#ifndef _BSPLINES_H_
#define _BSPLINES_H_

#include <vector>
#include <iostream>
#include <math.h>
#include <jrl/mal/matrixabstractlayer.hh>

struct Point
{
    double x;
    double y;
};

//namespace PatternGeneratorJRL
//{

  /** Bspline class */
  class  Bsplines
    {

    public:
        /*! Constructor */
        Bsplines(int degree, std::vector<Point> control_points);

        /*! Destructor */
        ~Bsplines();

        /*! Create a Knot Vector from m_degree and m_control_points with an algo "method" */
        void GenerateKnotVector(std::string method);

        /*! Create a derivative Bsplines*/
        Bsplines DerivativeBsplines();

        /*!Compute Basic Function */
        double *ComputeBasisFunction(double t);

        /*!Compute Bsplines */
        Point ComputeBplines(double t);

        /*! Set Degree */
        void SetDegree(int &degree);

        /*! Set Control Points */
        void SetControlPoints(std::vector<Point> &control_points) ;

        /*! Set Knot Vector */
        void SetKnotVector(std::vector<double> &knot_vector) ;

        /*! Get Degree */
        int GetDegree() const;

        /*! Get Control Points */
        std::vector<Point> GetControlPoints() const;

        /*! Get Knot Vector*/
        std::vector<double> GetKnotVector() const;

        void PrintKnotVector() const;

        void PrintControlPoints() const;

        void PrintDegree() const;

    protected:

        int m_degree;

        std::vector<Point> m_control_points;

        std::vector<double> m_knot_vector;
    };
#endif /* _BSPLINES_H_*/
//}
