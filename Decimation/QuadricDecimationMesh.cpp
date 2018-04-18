/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode
    QuadricDecimationMesh::QuadricIsoSurfaces =
        NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize() {
  // Allocate memory for the quadric array
  size_t numVerts = mVerts.size();
  mQuadrics.reserve(numVerts);
  std::streamsize width = std::cerr.precision(); // store stream precision
  for (size_t i = 0; i < numVerts; i++) {

    // Compute quadric for vertex i here
    mQuadrics.push_back(createQuadricForVert(i));

    // Calculate initial error, should be numerically close to 0

    Vector3<float> v0 = mVerts[i].pos;
    Vector4<float> v(v0[0], v0[1], v0[2], 1);
    Matrix4x4<float> m = mQuadrics.back();

    float error = v * (m * v);
    // std::cerr << std::scientific << std::setprecision(2) << error << " ";
  }
  std::cerr << std::setprecision(width) << std::fixed; // reset stream precision

  // Run the initialize for the parent class to initialize the edge collapses
  DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute,
 * DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse *collapse) {
  // Compute collapse->position and collapse->cost here
  // based on the quadrics at the edge endpoints
	Vector4<float> Ot{ 0.0f, 0.0f, 0.0f, 1.0f };
	//idices to vertices
	size_t v1 = mEdges[collapse->halfEdge].vert;
	size_t v2 = mEdges[mEdges[collapse->halfEdge].pair].vert;

	Matrix4x4<float> Q1 = mQuadrics[v1];
	Matrix4x4<float> Q2 = mQuadrics[v2];
	Matrix4x4<float> Q = Q1 + Q2;
	Matrix4x4<float> Qhat = Q;

	Qhat(3,0) = 0.0f;
	Qhat(3,1) = 0.0f;
	Qhat(3,2) = 0.0f;
	Qhat(3,3) = 1.0f;

	//Assumption: Q is invertible

	if (!Qhat.IsSingular()) {

		Qhat = Qhat.Inverse();

		Vector4<float> v = Qhat*Ot;
		collapse->position = Vector3<float>(v[0], v[1], v[2]);

		collapse->cost = v*(Q*v);
		//std::cout << v[0] << ","<< v[1] << "," << v[2] << "," << v[3] << " invert\n";
	}
	else {
		const Vector3<float> &v_0 = mVerts[mEdges[collapse->halfEdge].vert].pos;
		const Vector3<float> &v_1 = mVerts[mEdges[mEdges[collapse->halfEdge].pair].vert].pos;
		collapse->position = (v_0 + v_1) * 0.5;
		collapse->cost = (collapse->position - v_0).Length();
	}
	

 // std::cerr << "computeCollapse in QuadricDecimationMesh not implemented.\n";
}



/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(size_t ind) {
  DecimationMesh::updateVertexProperties(ind);
  mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForVert(size_t indx) const {
  float q[4][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}};
  Matrix4x4<float> Q(q);

  // The quadric for a vertex is the sum of all the quadrics for the adjacent
  // faces Tip: Matrix4x4 has an operator +=

  //Vector3<float> n = mVerts[indx].edge;

  std::vector<size_t> faces = HalfEdgeMesh::FindNeighborFaces(indx);

  for (auto f : faces) {
	  Q += createQuadricForFace(f);
  }

  return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
Matrix4x4<float>
QuadricDecimationMesh::createQuadricForFace(size_t indx) const {

	// Calculate the quadric (outer product of plane parameters) for a face
	// here using the formula from Garland and Heckbert

	//get normal
	Vector3<float> n = mFaces[indx].normal;
	//get d
	Vector3<float> v0 = mVerts[mEdges[mFaces[indx].edge].vert].pos; //point on surface

	float d = -(n*v0);

	Vector4<float> p{ n[0], n[1], n[2], d };

	float q[4][4] = { { p[0]*p[0], p[0] * p[1], p[0] * p[2], p[0] * p[3] },
					{ p[1] * p[0], p[1] * p[1], p[1] * p[2], p[1] * p[3] },
					{ p[2] * p[0], p[2] * p[1], p[2] * p[2], p[2] * p[3] },
					{ p[3] * p[0], p[3] * p[1], p[3] * p[2], p[3] * p[3] } };
	Matrix4x4<float> Q(q);
	return Q;
}

void QuadricDecimationMesh::Render() {
  DecimationMesh::Render();

  glEnable(GL_LIGHTING);
  glMatrixMode(GL_MODELVIEW);

  if (mVisualizationMode == QuadricIsoSurfaces) {
    // Apply transform
    glPushMatrix(); // Push modelview matrix onto stack

    // Implement the quadric visualization here
    std::cout << "Quadric visualization not implemented" << std::endl;

    // Restore modelview matrix
    glPopMatrix();
  }
}
