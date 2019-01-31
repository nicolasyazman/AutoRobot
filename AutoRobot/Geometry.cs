using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AutoRobot
{
    public class Geometry
    {

        /// <summary>
        /// This function computes the difference between two angles.
        /// </summary>
        /// <param name="b1">First angle, in degrees.</param>
        /// <param name="b2">Second angle, in degrees</param>
        /// <returns>Difference between the two angles.</returns>
        public static double Delta_Bearing(double b1, double b2)
        {
            double d = 0;

            // Convert bearing to W.C.B
            if (b1 < 0)
                b1 += 360;
            if (b2 < 0)
                b2 += 360;

            ///Calculate delta bearing
            //and
            //Convert result value to Q.B.
            d = (b2 - b1) % 360;

            if (d > 180)
                d -= 360;
            else if (d < -180)
                d += 360;

            return d;
        }
    }
}
