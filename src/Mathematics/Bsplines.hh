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

  /** Class for computing trajectories */
  class  Bspline
    {

    public:

        /*! Constructor */
        Bsplines();

        /*! Destructor */
        ~Bspline();

        /*!Set Parameters */
        SetBsplinesParameters(int order,std::vector<Point> control_points, std::vector<double> knot_vector) ;


        int GetOrder() const;

        std::vector<double> GetControlPoint() const;

        std::vector<double> GetKnotVector() const;
    private:

        int m_order;

        std::vector<Point> m_control_points;

        std::vector<double> m_knot_vector;
    };

}
