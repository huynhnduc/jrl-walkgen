/*
 * Copyright 2006, 2007, 2008, 2009, 2010,
 *
 * Olivier    Stasse
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
/* Polynomes object for generating foot trajectories. */
#include <iostream>
#include <vector>

#include <Debug.hh>
#include <Mathematics/PolynomeFoot.hh>
#include <Mathematics/Bsplines.hh>



using namespace::std;
using namespace::PatternGeneratorJRL;

ZBsplines::ZBsplines(double FT, double FP, double ToMP, double MP):Bsplines(4, GenerateControlPoints(FT, FP, ToMP, MP))
{
    ZGenerateKnotVector(FT, FP, ToMP, MP);
}

ZBsplines::~ZBsplines()
{

}

double ZBsplines::ZComputePosition(double t)
{
    if (t<m_FT)
        return ComputeBsplines(t).y;
    else
        return ComputeBsplines(m_FT - t).y;
}

double ZBsplines::ZComputeVelocity(double t)
{
    if (m_degree >=1){
        if (t<m_FT)
            return DerivativeBsplines().ComputeBsplines(t).y;
        else
            return DerivativeBsplines().ComputeBsplines(m_FT - t).y;
    }
    else
    {
        cout << "ERROR" << endl;
        return 0;
    }
}

double ZBsplines::ZComputeAcc(double t)
{
    if (m_degree >=2){
        if (t<m_FT)
            return DerivativeBsplines().DerivativeBsplines().ComputeBsplines(t).y;
        else
            return DerivativeBsplines().DerivativeBsplines().ComputeBsplines(m_FT - t).y;
    }
    else
    {
        cout << "ERROR" << endl;
        return 0;
    }
}

void ZBsplines::ZGenerateKnotVector(double FT, double FP, double ToMP, double MP)
{
    std::vector<double> knot;
    knot.clear();
    knot.push_back(0.0);
    knot.push_back(0.0);
    knot.push_back(0.0);
    knot.push_back(0.0);
    knot.push_back(m_FT*0.05);
    knot.push_back(0.85*ToMP);
    knot.push_back(1.15*ToMP);
    knot.push_back(1.25*ToMP);
    for (int i =3;i<=m_degree;i++)
    {
        knot.push_back(0.99999*FT);
    }
    knot.push_back(FT);
    knot.push_back(FT);
    knot.push_back(FT);
    SetKnotVector(knot);
}
std::vector<Point> ZBsplines::GenerateControlPoints(double FT, double FP, double ToMP, double MP)
{
    m_FT = FT;
    m_FP = FP;
    m_ToMP = ToMP;
    m_MP = MP;
    std::vector<Point> control_points;
    control_points.clear();
    std::ofstream myfile1;
    myfile1.open("control_point.txt");

    Point A = {0.0,0.0};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT*0.05,0.0};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT*0.1,0.0};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.85*m_ToMP,m_MP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {1.15*m_ToMP,m_MP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.85*m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {0.9*m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    A = {m_FT,m_FP};
    control_points.push_back(A);
    myfile1 << A.x <<" "<< A.y<< endl;

    myfile1.close();
return control_points;
}

Polynome3::Polynome3(double FT, double FP) :Polynome(3)
{
  SetParameters(FT,FP);
}

void Polynome3::SetParameters(double FT, double FP)
{
  m_FT = FT;
  m_FP = FP;
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  if(FP == 0.0 || FT == 0.0)
  {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
  }else{
    m_Coefficients[2] = 3.0*FP/tmp;
    m_Coefficients[3] = -2.0*FP/(tmp*FT);
  }
}

void Polynome3::SetParametersWithInitPosInitSpeed(double &FT,
						  double &FP,
						  double &InitPos,
						  double &InitSpeed)
{
  m_FT = FT;
  m_FP = FP;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  if(FT == 0.0)
  {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
  }else{
    m_Coefficients[2] = (3*FP - 3*InitPos - 2*InitSpeed*FT)/tmp;
    m_Coefficients[3] = (InitSpeed*FT+ 2*InitPos - 2*FP)/(tmp*FT);
  }
}

void Polynome3::GetParametersWithInitPosInitSpeed(double &FT,
						  double &FP,
						  double &InitPos,
						  double &InitSpeed)
{
  InitPos= m_Coefficients[0];
  InitSpeed= m_Coefficients[1];
  FT = m_FT;
  FP = m_FP;
}

Polynome3::~Polynome3()
{}

Polynome4::Polynome4(double FT, double FP) :Polynome(4)
{
  SetParameters(FT,FP);
}

void Polynome4::SetParameters(double FT, double MP)
{
  m_FT = FT;
  m_MP = MP;
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  tmp = FT*FT;
  if(MP==0.0 || tmp==0.0)
  {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
  }else{
    m_Coefficients[2] = 16.0*MP/tmp;
    tmp=tmp*FT;
    m_Coefficients[3] = -32.0*MP/tmp;
    tmp=tmp*FT;
    m_Coefficients[4] = 16.0*MP/tmp;
  }
}

void Polynome4::SetParametersWithInitPosInitSpeed(double FT,
						  double MP,
						  double InitPos,
						  double InitSpeed)
{
  m_FT = FT;
  m_MP = MP;

  double tmp;
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  tmp = FT*FT;
  if(tmp==0.0)
  {
    m_Coefficients[2] = 0.0;
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
  }else{
    m_Coefficients[2] = (-4.0*InitSpeed*FT - 11.0*InitPos + 16.0*MP)/tmp;
    tmp=tmp*FT;
    m_Coefficients[3] = ( 5.0*InitSpeed*FT + 18.0*InitPos - 32.0*MP)/tmp;
    tmp=tmp*FT;
    m_Coefficients[4] = (-2.0*InitSpeed*FT - 8.0 *InitPos + 16.0*MP)/tmp;
  }
}
void Polynome4::GetParametersWithInitPosInitSpeed(double &FT,
						  double &MP,
						  double &InitPos,
						  double &InitSpeed)
{
  FT = m_FT;
  MP = m_MP;
  InitPos = m_Coefficients[0];
  InitSpeed = m_Coefficients[1];
}
Polynome4::~Polynome4()
{}
Polynome5::Polynome5(double FT, double FP)
  :Polynome(5),
   FT_(FT),
   FP_(FP),
   InitPos_(0.0),
   InitSpeed_(0.0),
   InitAcc_(0.0)

{
  SetParameters(FT,FP);
}

Polynome5::~Polynome5()
{}

void Polynome5::SetParameters(double FT, double FP)
{
  double tmp;
  FT_ = FT; FP_=FP; InitPos_ = 0.0;
  InitSpeed_ = 0; InitAcc_ = 0.0;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  if(FP==0.0 || tmp==0.0)
  {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
  }else{
    m_Coefficients[3] = 10*FP/tmp;
    tmp *=FT;
    m_Coefficients[4] = -15*FP/tmp;
    tmp*=FT;
    m_Coefficients[5] = 6*FP/tmp;
  }
}

void Polynome5::SetParametersWithInitPosInitSpeed(double FT,
                                                  double FP,
                                                  double InitPos,
                                                  double InitSpeed)
{
  double tmp;
  m_Coefficients[0] = InitPos_ = InitPos;
  m_Coefficients[1] = InitSpeed_ = InitSpeed;
  InitPos_ = InitPos; InitSpeed = InitSpeed_; InitAcc_ = 0.0;
  m_Coefficients[2] = 0.0/2.0;
  tmp = FT*FT*FT;
  m_Coefficients[3] = (-3.0/2.0*0.0*FT*FT-6.0*InitSpeed*FT - 10.0*InitPos + 10.0*FP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = ( 3.0/2.0*0.0*FT*FT + 8.0*InitSpeed*FT + 15.0*InitPos - 15.0*FP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[5] = ( -1.0/2.0*0.0*FT*FT - 3.0*InitSpeed*FT - 6.0*InitPos + 6.0*FP)/tmp;
}

void Polynome5::GetParametersWithInitPosInitSpeed(double &FT,
                                                  double &FP,
                                                  double &InitPos,
                                                  double &InitSpeed)
{
  InitPos = m_Coefficients[0] ;
  InitSpeed = m_Coefficients[1];
  FT = FT_;
  FP = FP_;
}

void Polynome5::SetParameters(double FT, double FP,
    double InitPos, double InitSpeed, double InitAcc)
{
  double tmp;
  m_Coefficients[0] = InitPos_ = InitPos;
  m_Coefficients[1] = InitSpeed_ = InitSpeed;
  m_Coefficients[2] = InitAcc/2.0; InitAcc_ = InitAcc;
  FT_ = FT; FP_ = FP;
  tmp = FT*FT*FT;
  m_Coefficients[3] = (-3.0/2.0*InitAcc*FT*FT-6.0*InitSpeed*FT - 10.0*InitPos + 10.0*FP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[4] = ( 3.0/2.0*InitAcc*FT*FT + 8.0*InitSpeed*FT + 15.0*InitPos - 15.0*FP)/tmp;
  tmp=tmp*FT;
  m_Coefficients[5] = ( -1.0/2.0*InitAcc*FT*FT - 3.0*InitSpeed*FT - 6.0*InitPos + 6.0*FP)/tmp;
}

Polynome6::Polynome6(double FT, double MP) :Polynome(6)
{
  SetParameters(FT,MP);
}

void Polynome6::SetParameters(double FT, double MP)
{
  double tmp;
  m_Coefficients[0] = 0.0;
  m_Coefficients[1] = 0.0;
  m_Coefficients[2] = 0.0;
  tmp = FT*FT*FT;
  if(MP==0.0 || tmp==0.0)
  {
    m_Coefficients[3] = 0.0;
    m_Coefficients[4] = 0.0;
    m_Coefficients[5] = 0.0;
    m_Coefficients[6] = 0.0;
  }else{
    m_Coefficients[3] = 64*MP/tmp;
    tmp *=FT;
    m_Coefficients[4] = -192*MP/tmp;
    tmp *=FT;
    m_Coefficients[5] = 192*MP/tmp;
    tmp *=FT;
    m_Coefficients[6] = -64*MP/tmp;
  }
}


void Polynome6::SetParameters(
		double FT, double PM,
		double InitPos, double InitSpeed, double InitAcc)
{
  m_Coefficients[0] = InitPos;
  m_Coefficients[1] = InitSpeed;
  m_Coefficients[2] = 0.5*InitAcc;
  m_Coefficients[3] = -0.5*(5*FT*FT*InitAcc + 32*InitSpeed*FT + 84*InitPos - 128*PM)/(FT*FT*FT);
  m_Coefficients[4] =  0.5*(76*InitSpeed*FT + 222*InitPos - 384*PM + 9*FT*FT*InitAcc)/(FT*FT*FT*FT);
  m_Coefficients[5] = -0.5*(204*InitPos + 66*InitSpeed*FT - 384*PM + 7*FT*FT*InitAcc)/(FT*FT*FT*FT*FT);
  m_Coefficients[6] =      (-64*PM+32*InitPos + 10*InitSpeed*FT + FT*FT*InitAcc)/(FT*FT*FT*FT*FT*FT);
}

Polynome6::~Polynome6()
{
}

