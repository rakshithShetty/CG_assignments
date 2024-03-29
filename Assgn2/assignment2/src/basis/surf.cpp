#include "surf.h"
#include "extra.h"

using namespace std;
using namespace FW;

namespace
{
    // This is a generic function that generates a set of triangle
    // faces for a sweeping a profile curve along "something".  For
    // instance, say you want to sweep the profile curve [01234]:
    //
    //   4     9     10
    //    3     8     11
    //    2 --> 7 --> 12 ----------[in this direction]--------->
    //    1     6     13 
    //   0     5     14
    //
    // Then the "diameter" is 5, and the "length" is how many times
    // the profile is repeated in the sweep direction.  This function
    // generates faces in terms of vertex indices.  It is assumed that
    // the indices go as shown in the picture (the first dia vertices
    // correspond to the first repetition of the profile curve, and so
    // on).  It will generate faces [0 5 1], [1 5 6], [1 6 2], ...
    // The boolean variable "closed" will determine whether the
    // function closes the curve (that is, connects the last profile
    // to the first profile).
    static vector< FW::Vec3i > triSweep( unsigned dia, unsigned len, bool closed )
    {
        vector< FW::Vec3i > ret;

		// YOUR CODE HERE: generate zigzagging triangle indices and push them to ret.
		for (int i = 0; i < (len - 1); i++){
			for (int j = 0; j < dia - 1; j++){
				// Each step we handle one quadrilateral and break it into 2 triangles
				// Hence 2*dia vertices make dia - 1 quads and 2(dia -1) triangles
				// Triangle 1
				ret.push_back(Vec3i(i*dia + j, (i + 1)*dia + j, i*dia + j + 1));				
				// Triangle 2
				ret.push_back(Vec3i(i*dia + j + 1, (i + 1)*dia + j, (i+1)*dia + j + 1));
			}
		}
		if (closed){
			int i = len - 1;
			for (int j = 0; j < dia - 1; j++){
				// Triangle 1
				ret.push_back(Vec3i(i*dia + j, j, i*dia + j + 1));
				// Triangle 2
				ret.push_back(Vec3i(i*dia + j + 1, j, j + 1));
			}
		}
        return ret;
    }
    
    // We're only implenting swept surfaces where the profile curve is
    // flat on the xy-plane.  This is a check function.
    static bool checkFlat(const Curve &profile)
    {
        for (unsigned i=0; i<profile.size(); i++)
            if (profile[i].V[2] != 0.0 ||
                profile[i].T[2] != 0.0 ||
                profile[i].N[2] != 0.0)
                return false;
    
        return true;
    }
}

Surface makeSurfRev(const Curve &profile, unsigned steps)
{
    Surface surface;
    
    if (!checkFlat(profile))
    {
        cerr << "surfRev profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // YOUR CODE HERE: build the surface.  See surf.h for type details.
	// Generate the vertices and normals by looping the number of the steps and again for each 
	// point in the profile (that's two cascaded loops), and finally get the faces with triSweep.
	// You'll need to rotate the curve at each step, similar to the cone in assignment 0 but
	// now you should be using a real rotation matrix.
	float step_angle = 2 * FW_PI /steps;

    //cerr << "\t>>> makeSurfRev called (but not implemented).\n\t>>> Returning empty surface." << endl;
	Mat4f Rot = rotation4f(Vec3f(0, 1, 0), -step_angle);
	
	for (int i = 0; i < steps; i++) {
		for (int j = 0; j < profile.size(); j++){
			if (i == 0){
				surface.VV.push_back(profile[j].V);
				surface.VN.push_back(-profile[j].N);
			}
			else{
				surface.VV.push_back(Rot * surface.VV[(i - 1)*profile.size() + j]);
				surface.VN.push_back(Rot * surface.VN[(i - 1)*profile.size() + j]);
			}
		}
	}
	surface.VF = triSweep(profile.size(), steps, TRUE);
 
    return surface;
}

Surface makeGenCyl(const Curve &profile, const Curve &sweep )
{
    Surface surface;

    if (!checkFlat(profile))
    {
        cerr << "genCyl profile curve must be flat on xy plane." << endl;
        exit(0);
    }

    // YOUR CODE HERE: build the surface. 
	// This is again two cascaded loops. Build the local coordinate systems and transform
	// the points in a very similar way to the one with makeSurfRev.

    //cerr << "\t>>> makeGenCyl called (but not implemented).\n\t>>> Returning empty surface." <<endl;
	Mat4f trans;
	Mat3f Rot;
	cout << "Total sizes are " << sweep.size() <<" and " <<profile.size();
	for (int i = sweep.size()-1; i >= 0; i--) {
		trans.setCol(0, Vec4f(sweep[i].N, 0));
		trans.setCol(1, Vec4f(sweep[i].B, 0));
		trans.setCol(2, Vec4f(sweep[i].T, 0));
		trans.setCol(3, Vec4f(sweep[i].V, 1));
		
		Rot.setCol(0,sweep[i].N);
		Rot.setCol(1, sweep[i].B);
		Rot.setCol(2, sweep[i].T);
		
		/*cout << "Now in point: " << i << endl;
		printTranspose(trans.getRow(0)); cout << endl;
		printTranspose(trans.getRow(1)); cout << endl;
		printTranspose(trans.getRow(2)); cout << endl;
		printTranspose(trans.getRow(3)); cout << endl;*/
		for (int j = 0; j < profile.size(); j++) {
			surface.VV.push_back(trans * profile[j].V);
			surface.VN.push_back(Rot * -profile[j].N);
		}
	}
	surface.VF = triSweep(profile.size(), sweep.size() - 1, TRUE);
	cout << "We are done here!!: " << surface.VF.size() << endl;
    return surface;
}

void drawSurface(const Surface &surface, bool shaded)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    if (shaded)
    {
        // This will use the current material color and light
        // positions.  Just set these in drawScene();
        glEnable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        // This tells openGL to *not* draw backwards-facing triangles.
        // This is more efficient, and in addition it will help you
        // make sure that your triangles are drawn in the right order.
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
    }
    else
    {        
        glDisable(GL_LIGHTING);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        
        glColor4f(0.4f,0.4f,0.4f,1.f);
        glLineWidth(1);
    }

    glBegin(GL_TRIANGLES);
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        glNormal(surface.VN[surface.VF[i][0]]);
        glVertex(surface.VV[surface.VF[i][0]]);
        glNormal(surface.VN[surface.VF[i][1]]);
        glVertex(surface.VV[surface.VF[i][1]]);
        glNormal(surface.VN[surface.VF[i][2]]);
        glVertex(surface.VV[surface.VF[i][2]]);
    }
    glEnd();

    glPopAttrib();
}

void drawNormals(const Surface &surface, float len)
{
    // Save current state of OpenGL
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_LIGHTING);
    glColor4f(0,1,1,1);
    glLineWidth(1);

    glBegin(GL_LINES);
    for (unsigned i=0; i<surface.VV.size(); i++)
    {
        glVertex(surface.VV[i]);
        glVertex(surface.VV[i] + surface.VN[i] * len);
    }
    glEnd();

    glPopAttrib();
}

void outputObjFile(ostream &out, const Surface &surface)
{
    
    for (unsigned i=0; i<surface.VV.size(); i++)
        out << "v  "
            << surface.VV[i][0] << " "
            << surface.VV[i][1] << " "
            << surface.VV[i][2] << endl;

    for (unsigned i=0; i<surface.VN.size(); i++)
        out << "vn "
            << surface.VN[i][0] << " "
            << surface.VN[i][1] << " "
            << surface.VN[i][2] << endl;

    out << "vt  0 0 0" << endl;
    
    for (unsigned i=0; i<surface.VF.size(); i++)
    {
        out << "f  ";
        for (unsigned j=0; j<3; j++)
        {
            unsigned a = surface.VF[i][j]+1;
            out << a << "/" << "1" << "/" << a << " ";
        }
        out << endl;
    }
}
