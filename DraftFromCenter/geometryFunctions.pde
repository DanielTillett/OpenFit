// Rotate a vector in 2D
void rotate2D(PVector v, float angle) {
  float theta = map(angle, 0, 360, 0, TWO_PI);
  // What's the magnitude?
  float m = v.mag();
  // What's the angle?
  float a = v.heading();
  // Change the angle
  a += theta;
  // Polar to cartesian for the new xy components
  v.x = m * cos(a);
  v.y = m * sin(a);
}

float bezierLength(PVector a, PVector cp1, PVector cp2, PVector b) {
  float l = 0;
  float precision = .01;
  for (int i = 0; i <= 1/precision; i++) { // 1/precision is a float? works?)
    float t = i*precision;
    float x1 = bezierPoint(a.x, cp1.x, cp2.x, b.x, t);
    float x2 = bezierPoint(a.x, cp1.x, cp2.x, b.x, t+precision);
    float y1 = bezierPoint(a.y, cp1.y, cp2.y, b.y, t);
    float y2 = bezierPoint(a.y, cp1.y, cp2.y, b.y, t+precision);
    l += dist(x1, y1, x2, y2);
  }
  return l;
}

// a and b points on line, c is center of circle, r is radius
// posNeg if need to toggle between two solutions
PVector circleIsecLine(PVector a, PVector b, PVector c, float r, boolean posNeg) {
  float slope = (b.y - a.y)/(b.x - a.x);
  float yintercept = b.y - (slope*b.x);
  PVector isec = new PVector(0, 0);
  if (posNeg == true) {
    isec.x = - sqrt( 
    - sq(yintercept) - (2.*yintercept*c.x*slope) + (2.*yintercept*c.y) - (sq(c.x)*sq(slope)) + 
      (2.*c.x*c.y*slope) - sq(c.y) + (sq(slope)*sq(r)) + sq(r)
      ) 
      - (yintercept* slope) + c.x + (c.y* slope);
  } 
  else {
    isec.x = sqrt( 
    - sq(yintercept) - (2.*yintercept*c.x*slope) + (2.*yintercept*c.y) - (sq(c.x)*sq(slope)) + 
      (2.*c.x*c.y*slope) - sq(c.y) + (sq(slope)*sq(r)) + sq(r)
      ) 
      - (yintercept* slope) + c.x + (c.y* slope);
  }
  isec.x /= sq(slope) + 1.; 
  isec.y = slope* isec.x + yintercept;
  return new PVector (isec.x, isec.y);
}

// bezier is a, cp1, cp2, b; find the point where the curve crosses y = targetY
// assumes y changes monotonically along the curve (true for the crotch curves).
// if the curve never reaches targetY, returns the nearer end of the curve
PVector bezierPointAtY(PVector a, PVector cp1, PVector cp2, PVector b, float targetY) {
  if (targetY < min(a.y, b.y) || targetY > max(a.y, b.y)) {
    println("bezierPointAtY(): the curve doesn't reach y = " + targetY + ", using its end");
    return abs(a.y - targetY) < abs(b.y - targetY) ? a.get() : b.get();
  }
  float lo = 0, hi = 1;
  boolean rising = b.y > a.y;
  for (int i = 0; i < 50; i++) {
    float t = (lo + hi) / 2.;
    float y = bezierPoint(a.y, cp1.y, cp2.y, b.y, t);
    if ((y < targetY) == rising) {
      lo = t;
    } else {
      hi = t;
    }
  }
  float t = (lo + hi) / 2.;
  return new PVector(bezierPoint(a.x, cp1.x, cp2.x, b.x, t), targetY);
}

PVector intersectAtDistance(PVector a, PVector b, PVector c, PVector d) {
  float maxScale = 10; // line has to be within this normalized distance
  Line2D ab = makeLine(a, b);
  Line2D cd = makeLine(c, d);
  ab.offsetAndGrowBy(0, maxScale, ab.getMidPoint());
  cd.offsetAndGrowBy(0, maxScale, cd.getMidPoint());
  Line2D.LineIntersection isec = ab.intersectLine(cd);
  Vec2D pos = isec.getPos();
  return new PVector(pos.x, pos.y);
}

Line2D makeLine(PVector a, PVector b) {
  Vec2D va = new Vec2D(a.x, a.y);
  Vec2D vb = new Vec2D(b.x, b.y);
  return new Line2D(va, vb);
}

//distance in relation to point 2
PVector pointOnLine(PVector point1, PVector point2, float distance, boolean posNeg) { 
  float slope = (point2.y - point1.y)/(point2.x - point1.x);
  float yintercept = point2.y - (slope * point2.x);
  PVector isec = new PVector (0, 0);
  if (posNeg == true) {
    isec.x = - sqrt(
    - sq(yintercept) - (2.*yintercept*point2.x*slope) + (2.*yintercept*point2.y) + 
      (sq(distance)*sq(slope)) + sq(distance) - (sq(point2.x)*sq(slope)) +
      (2.*point2.x*point2.y*slope) - sq(point2.y)
      ) - (yintercept*slope) + point2.x + (point2.y*slope);
  } else {
    isec.x = sqrt(
    - sq(yintercept) - (2.*yintercept*point2.x*slope) + (2.*yintercept*point2.y) + 
      (sq(distance)*sq(slope)) + sq(distance) - (sq(point2.x)*sq(slope)) +
      (2.*point2.x*point2.y*slope) - sq(point2.y)
      ) - (yintercept*slope) + point2.x + (point2.y*slope);
  } 
  isec.x /= sq(slope) +1.;
  isec.y = (isec.x*slope) + yintercept;
  return new PVector (isec.x, isec.y);
}

// seam allowance functions:

// generate parallel lines from two points, doesn't work with horizontal lines
// a and b are two points, seam allowance is distance, toggle between new points c and d, toggle direction
PVector parallelLinePoint(PVector a, PVector b, float seamAllowance, boolean togglePoint, boolean posNeg) {
  float slope = (b.y - a.y)/(b.x - a.x);
  float yintercept = b.y - (slope*b.x);
  float perpSlope = -(1/slope);
  float aYintercept = a.y - (perpSlope*a.x); // y intercept for perpendicular from point a
  float bYintercept = b.y - (perpSlope*b.x); // y intercept for perpendicular from point b
  PVector c = new PVector(0, 0);
  PVector d = new PVector(0, 0);
  // find the points along perpendicular slope, seam allowance is distance
  if (posNeg == true) {
    c.x = - sqrt(
    - sq(aYintercept) - (2.*aYintercept*a.x*perpSlope) + (2.*aYintercept*a.y) + 
      (sq(seamAllowance)*sq(perpSlope)) + sq(seamAllowance) - (sq(a.x)*sq(perpSlope)) +
      (2.*a.x*a.y*perpSlope) - sq(a.y)
      ) - (aYintercept*perpSlope) + a.x + (a.y*perpSlope);
    d.x = - sqrt(
    - sq(bYintercept) - (2.*bYintercept*b.x*perpSlope) + (2.*bYintercept*b.y) + 
      (sq(seamAllowance)*sq(perpSlope)) + sq(seamAllowance) - (sq(b.x)*sq(perpSlope)) +
      (2.*b.x*b.y*perpSlope) - sq(b.y)
      ) - (bYintercept*perpSlope) + b.x + (b.y*perpSlope);
  } else {
    c.x = sqrt(
    - sq(aYintercept) - (2.*aYintercept*a.x*perpSlope) + (2.*aYintercept*a.y) + 
      (sq(seamAllowance)*sq(perpSlope)) + sq(seamAllowance) - (sq(a.x)*sq(perpSlope)) +
      (2.*a.x*a.y*perpSlope) - sq(a.y)
      ) - (aYintercept*perpSlope) + a.x + (a.y*perpSlope);
    d.x = sqrt(
    - sq(bYintercept) - (2.*bYintercept*b.x*perpSlope) + (2.*bYintercept*b.y) + 
      (sq(seamAllowance)*sq(perpSlope)) + sq(seamAllowance) - (sq(b.x)*sq(perpSlope)) +
      (2.*b.x*b.y*perpSlope) - sq(b.y)
      ) - (bYintercept*perpSlope) + b.x + (b.y*perpSlope);
  }
  c.x /= sq(perpSlope) +1.;
  c.y = (c.x*perpSlope) + aYintercept; 
  d.x /= sq(perpSlope) +1.;
  d.y = (d.x*perpSlope) + bYintercept; 
  if (togglePoint == true) {
    return new PVector(c.x, c.y);
  } else {
    return new PVector(d.x, d.y);
  }
} 

// shift control point toward outside of pattern amount of seam allowance
/* a and b are points that lie on a line perpendicular to the one 
 that we shift the control, c is the control point, seam allowance is the distance,
 toggle direction
 */
PVector shiftControlPoint (PVector a, PVector b, PVector c, float seamAllowance, boolean posNeg) {
  float parallelSlope = (b.y - a.y)/(b.x - a.x);
  float slope = -(1/parallelSlope); // slope that control point lies along
  float yintercept = c.y -(slope*c.x);
  PVector d = new PVector(0, 0);
  if (posNeg == true) {
    d.x = - sqrt(
    - sq(yintercept) - (2.*yintercept*c.x*slope) + (2.*yintercept*c.y) + 
      (sq(seamAllowance)*sq(slope)) + sq(seamAllowance) - (sq(c.x)*sq(slope)) +
      (2.*c.x*c.y*slope) - sq(c.y)
      ) - (yintercept*slope) + c.x + (c.y*slope);
  } else {
    d.x = sqrt(
    - sq(yintercept) - (2.*yintercept*c.x*slope) + (2.*yintercept*c.y) + 
      (sq(seamAllowance)*sq(slope)) + sq(seamAllowance) - (sq(c.x)*sq(slope)) +
      (2.*c.x*c.y*slope) - sq(c.y)
      ) - (yintercept*slope) + c.x + (c.y*slope);
  }
  d.x /= sq(slope) +1.;
  d.y = (d.x*slope) + yintercept; 
  return new PVector(d.x, d.y);
}

