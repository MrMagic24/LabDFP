using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Drawing;
using System.Threading.Tasks;

namespace LabDFP
{
    public struct Response
    {
        public PointF Arg { get; set; }
        public double Func { get; set; }
        public int Iterations { get; set; }
    }

    class MainMethod
    {
        public double Function(PointF x)
        {
            return Math.Pow(x.X - 2, 2) + Math.Pow(x.Y - 3, 2);
            //return 2 * Math.Pow(x.X, 2) + x.X * x.Y + Math.Pow(x.Y, 2);
        }

        private double FunctionGrad1(PointF x)
        {
            // (x^2 - 4x + 4) + (y^2 - 6y + 9)
            //return 4 * x.X + x.Y;
            return 2 * x.X - 4;
        }

        private double FunctionGrad2(PointF x)
        {
            //return x.X + 2 * x.Y;
            return 2 * x.Y - 6;
        }

        private double Grad(int i, PointF x)
        {
            switch (i)
            {
                case 1:
                    return FunctionGrad1(x);
                case 2:
                    return FunctionGrad2(x);
            }

            throw new ArgumentException("Ошибка!");
        }

        private double StepResolver(double x1, double x2, double c1, double c2)
        {
            return (4 * x1 * c1 + x1 * c2 + x2 * c1 + 2 * x2 * c2) /
                   (4 * c1 * c1 + 2 * c1 * c2 + 2 * c2 * c2);
        }

        private PointF _x, _xPrev, _xPrevPrev, deltaX;
        private double _e1, _e2, _t;
        private int k, _m;
        private double[] fg, d, dg;
        private double[,] A, Ac;

        public Response FindMinValue(double x0, double y0, double e1, double e2, int m)
        {
            A = new double[,] { { 1, 0 }, { 0, 1 } };
            d = new double[2];
            dg = new double[2];
            _x = new PointF(Convert.ToSingle(x0), Convert.ToSingle(y0));
            _xPrev = _x;
            k = 0;
            _m = m;
            _e1 = e1;
            _e2 = e2;

            
            THREE:
            fg = new[]
            {
                Grad(1, _x),
                Grad(2, _x)
            };


            FOUR:
            double fgLength = (double)Math.Sqrt(fg[0] * fg[0] + fg[1] * fg[1]);

            if (fgLength < _e1)
            {
                goto END;
            }

            FIVE:
            if (k >= _m)
            {
                goto END;
            }

            if (k > 0)
            {
                goto SIX;
            }

            else
            {
                goto TEN;
            }

            SIX:
            double[] fgPrev = { Grad(1, _xPrev), Grad(2, _xPrev) };

            dg = new[] { fg[0] - fgPrev[0], fg[1] - fgPrev[1] };

            SEVEN:
            deltaX = new PointF(_x.X - _xPrev.X, _x.Y - _xPrev.Y);

            EIGHT:
            double c1 = deltaX.X * dg[0] + deltaX.Y * dg[1];

            double[,] At1 =
            {
                {deltaX.X * deltaX.X / c1, deltaX.X * deltaX.Y / c1},
                {deltaX.Y * deltaX.X / c1, deltaX.Y * deltaX.Y / c1}
            };

            PointF v1 = new PointF
            {
                X = Convert.ToSingle(A[0, 0] * dg[0] + A[0, 1] * dg[1]),
                Y = Convert.ToSingle(A[1, 0] * dg[0] + A[1, 1] * dg[1])
            };

            double[,] At2 =
            {
                {v1.X * dg[0], v1.X * dg[1]},
                {v1.Y * dg[0], v1.Y * dg[1]}
            };

            PointF v2 = new PointF
            {
                X = Convert.ToSingle(dg[0] * A[0, 0] + dg[1] * A[1, 0]),
                Y = Convert.ToSingle(dg[1] * A[0, 1] + dg[1] * A[1, 1])
            };

            double c2 = v1.X * dg[0] + v1.Y * dg[1];

            double[,] At3 =
            {
                {
                    (At2[0, 0] * A[0, 0] + At2[0, 1] * A[1, 0]) / c2,
                    (At2[0, 0] * A[0, 1] + At2[0, 1] * A[1, 1]) / c2
                },
                {
                    (At2[1, 0] * A[0, 0] + At2[1, 1] * A[1, 0]) / c2,
                    (At2[1, 0] * A[0, 1] + At2[1, 1] * A[1, 1]) / c2
                }
            };

            Ac = new[,]
            {
                {At1[0, 0] - At3[0, 0], At1[0, 1] - At3[0, 1]},
                {At1[1, 0] - At3[1, 0], At1[1, 1] - At3[1, 1]}
            };

            NINE:
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    A[i, j] += Ac[i, j];
                }
            }

            TEN:
            d = new[]
            {
                -(A[0, 0] * fg[0] + A[0, 1] * fg[1]),
                -(A[1, 0] * fg[0] + A[1, 1] * fg[1])
            };

            ELEVEN:
            _t = StepResolver(_x.X, _x.Y, d[0], d[1]);

            TWELVE:
            _xPrev = _x;

            PointF temp12 = new PointF
            {
                X = Convert.ToSingle(_x.X - _t * d[0]),
                Y = Convert.ToSingle(_x.Y - _t * d[1])
            };

            _x = temp12;

            THIRTEEN:
            PointF temp13 = new PointF
            {
                X = _x.X - _xPrev.X,
                Y = _x.Y - _xPrev.Y
            };

            double tempL = (double)Math.Sqrt(temp13.X * temp13.X + temp13.Y * temp13.Y);
            double F = Function(_x);
            double F1 = Function(_xPrev);

            if (tempL < _e2 && Math.Abs(Function(_x) - Function(_xPrev)) < _e2)
            {
                goto END;
            }

            k++;
            goto THREE;

            END:
            return new Response
            {
                Arg = _x,
                Func = Function(_x),
                Iterations = k
            };
        }
    }
}
