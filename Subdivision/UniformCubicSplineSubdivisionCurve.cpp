
#include "UniformCubicSplineSubdivisionCurve.h"

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(
    const std::vector<Vector3<float>> &joints, Vector3<float> lineColor,
    float lineWidth)
    : mCoefficients(joints), mControlPolygon(joints) {
  this->mLineColor = lineColor;
  this->mLineWidth = lineWidth;
}

void UniformCubicSplineSubdivisionCurve::Subdivide() {
  // Allocate space for new coefficients
  std::vector<Vector3<float>> newc;

  assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

  // Implement the subdivision scheme for a natural cubic spline here
  //Where do we start??

  //1. find new points to subdivide the curve with

  //Insert the first boundary coefficient from mCoefficients into newc
  newc.push_back(mCoefficients[0]);

  //Calculate the new coefficients and push them to newc
  for (int i = 1; i < mCoefficients.size()-1; ++i) {

	  Vector3<float> ci = (mCoefficients[i - 1] + 6.0f*mCoefficients[i] + mCoefficients[i + 1]) / 8.0f;
	  Vector3<float> cihalf = (mCoefficients[i] + mCoefficients[i + 1]) / 2.0f;

	  newc.push_back(ci);
	  newc.push_back(cihalf);
  }

  //Insert end point as it is. It should not change
  newc.push_back(mCoefficients.back());

  // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check
  // here!
  assert(newc.size() != 2.0f*mCoefficients.size() - 1 && "Incorrect number of new coefficients!");

  mCoefficients = newc;
}

void UniformCubicSplineSubdivisionCurve::Render() {
  // Apply transform
  glPushMatrix(); // Push modelview matrix onto stack

  // Convert transform-matrix to format matching GL matrix format
  // Load transform into modelview matrix
  glMultMatrixf(mTransform.ToGLMatrix().GetArrayPtr());

  mControlPolygon.Render();

  // save line point and color states
  glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

  // draw segments
  glLineWidth(mLineWidth);
  glColor3fv(mLineColor.GetArrayPtr());
  glBegin(GL_LINE_STRIP);
  // just draw the spline as a series of connected linear segments
  for (size_t i = 0; i < mCoefficients.size(); i++) {
    glVertex3fv(mCoefficients.at(i).GetArrayPtr());
  }
  glEnd();

  // restore attribs
  glPopAttrib();

  glPopMatrix();

  GLObject::Render();
}
