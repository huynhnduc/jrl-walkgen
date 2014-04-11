/** \file StepOverPolynome.h
    \brief Polynomes object for generating foot and hip trajectories while stepping over. */


#ifndef _BSPLINES_H_
#define _BSPLINES_H_

#include <vector>

#include <jrl/mal/matrixabstractlayer.hh>


#include <Mathematics/Polynome.hh>

namespace PatternGeneratorJRL
{

  /** Class for computing trajectories */
  class  Bspline
    {

    public:

      /*! Constructor */
      Bspline(int order,);

      /*! Destructor */
      ~Bspline();
    }
