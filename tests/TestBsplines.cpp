/*
 * Copyright 2009, 2010, 2014
 *
 * 
 * Olivier  Stasse, Huynh Ngoc Duc
 *
 * JRL, CNRS/AIST
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Research carried out within the scope of the
 *  Joint Japanese-French Robotics Laboratory (JRL)
 */
/*! \file TestBsplines.cpp
  \brief This Example shows you how to use Bsplines to create a foot trajectory on Z . */

#include <iostream>
#include <fstream>
#include "Mathematics/PolynomeFoot.hh"

using namespace std;

int main()
{
    double t=0.0;
    int m_degree;
    int i , j;
    ofstream myfile;
    myfile.open("test.txt");

    double m_FT = 0.7;
    double m_FP = 0.2;
    double m_MP = 0.3;
    double m_ToMP = m_FT/3.0;

    PatternGeneratorJRL::ZBsplines Z(m_FT, m_FP, m_ToMP, m_MP);

    Z.PrintDegree();
    Z.PrintKnotVector();
    Z.PrintControlPoints();

    for (int k=1; k<1000;k++)
    {

        t = double(k)*Z.GetKnotVector().back()/1000.0;
        cout << k << endl;
        myfile << t << " " << Z.ZComputePosition(t)<<" "<< Z.ZComputeVelocity(t)<< " "<< Z.ZComputeAcc(t)<< endl;
        cout <<  t  << " " << Z.ZComputePosition(t)<<" "<< Z.ZComputeVelocity(t)<< " "<< Z.ZComputeAcc(t)<< endl;
    }
    myfile.close();
    return 0;
}
