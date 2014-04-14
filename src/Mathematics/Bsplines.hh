/** \file StepOverPolynome.h
    \brief Polynomes object for generating foot and hip trajectories while stepping over. */


#ifndef _BSPLINES_H_
#define _BSPLINES_H_

#include <vector>

#include <jrl/mal/matrixabstractlayer.hh>

#include <Mathematics/Polynome.hh>

struct Point
{
    double x;
    double y;
}

namespace PatternGeneratorJRL
{

  /** Bspline class */
  class  Bspline
    {

    public:

        /*! Constructor */
        Bsplines();

        /*! Destructor */
        ~Bspline();

        /*!Set Parameters of Basis Function*/
        void SetBasisFunctionParameters(int order,
                              std::vector<double> knot_vector) ;

        double **ComputeBasisFunction(int t);

        void Bsplines::SetOrder(int order);

        void Bsplines::SetControlPoints(std::vector<Point> control_points) ;

        void Bsplines::SetKnotVector(std::vector<double> knot_vector);

        int GetOrder() const;

        std::vector<Point> GetControlPoints() const;

        std::vector<double> GetKnotVector() const;

    protected:

        int m_order;

        std::vector<Point> m_control_points;

        std::vector<double> m_knot_vector;
    };

}
