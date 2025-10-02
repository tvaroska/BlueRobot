package frc.robot;

/**
 *
 */
class Sensitivity {
    double threshold;
    double zeroValue;
    double linCoef;
    double cuspX;
    double limit;
    double a, b, c;

    public Sensitivity(double th, double zv, double cx, double lc, double li) {
        set(th, zv, cx, lc, li);
    }

    public void set(double th, double zv, double cx, double lc, double li) {
        zeroValue = zv;
        threshold = th;
        cuspX = cx;
        linCoef = lc;
        limit = li;

        double x0 = cuspX;
        double g = (2.0 - x0) * x0;
        double denom = 1.0 - g;
        a = (1.0 - linCoef) / denom;
        c = a * x0 * x0;
        b = (linCoef + (linCoef * x0 - 2.0) * x0) / denom;
    }

    double transfer(double x) {
        double xabs = Math.abs(x);
        if (threshold < 0) {
            xabs -= threshold;
            if (xabs <= 0)
              return 0;
        }
        else if (xabs < threshold)
          return 0;

        if (xabs <= cuspX) {
            //System.out.println("Lin " + m_linCoef + " x: " + x);
            xabs *= linCoef;
        }
        else {//if (xabs <= 1.0) {
            //System.out.println("Quad " + m_threshold + " x: " + x);
            xabs = a * xabs * xabs + b * xabs + c;
        }
//        xabs += zeroValue;
        if (xabs > limit)
            xabs = limit;
        return x >= 0 ? xabs : -xabs;
    }
}
/*
double sensitivity(double x) {
    //m_speedLimit1 = SmartDashboard.getNumber("Drive speed limit1", 0.1);

    double xabs = Math.abs(x);
    if (xabs <= m_threshold) {
      return m_linCoef * x;
    }
    else {//if (xabs <= 1.0) {
      // double x2 = (xabs - m_speedStep1);
      // x2 *= x2;

      double c = m_linCoef * m_cuspX;
      double coef = (1 - c) / (1 - m_threshold);
      coef *= m_speedLimitX;
      xabs = coef * (xabs - m_threshold) + c;
      x = x >= 0 ? xabs : -xabs;
    }
    // else if (xabs <= 0.6) {
    //   // 0.32 = 0.4 * k
    //   //x = 0.4 * (x - 0.2) + 0.04;
    // }
    // else {
    //   // 0.2
    //   //x = m_speedLimit1 * (x - 0.6) + 0.2;
    // }
    return x;
  }

  static double sensitivity3(double x) {
    double xabs = Math.abs(x);
    if (xabs < 0.1)
      return 0;
    else if (xabs < 0.6)
      return 0.4 * Math.signum(x) * (xabs);
    else return 0;
    //return 0.5 * x * x * x;
  }

  void teleopInit() {
    m_roller.stop();
  }
}
  */