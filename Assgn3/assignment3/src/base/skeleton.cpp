#include "skeleton.hpp"
#include "utility.hpp"

#include <cassert>
#include <fstream>

using namespace std;
using namespace FW;

void Skeleton::load(string skeleton_file) {	
	ifstream in(skeleton_file);
	Joint joint;
	Vec3f pos;
	int parent;
	string name;
	unsigned current_joint = 0;
	while (in.good()) {
		in >> pos.x >> pos.y >> pos.z >> parent >> name;
		joint.name = name;
		joint.parent = parent;
		// set position of joint in parent's space
		joint.position = pos;
		if (in.good()) {
			joints_.push_back(joint);
			if (current_joint == 0) {
				assert(parent == -1 && "first node read should always be the root node");
			} else {
				assert(parent != -1 && "there should not be more than one root node");
				joints_[parent].children.push_back(current_joint);
			}
			++current_joint;
		}
	}

	// initially set to_parent matrices to identity
	for (auto j = 0u; j < joints_.size(); ++j)
		setJointRotation(j, Vec3f(0, 0, 0));

	// this needs to be done while skeleton is still in bind pose
	computeToBindTransforms();
}

string Skeleton::getJointName(unsigned index) const {
	return joints_[index].name;
}

Vec3f Skeleton::getJointRotation(unsigned index) const {
	return joints_[index].rotation;
}

int Skeleton::getJointParent(unsigned index) const {
	return joints_[index].parent;
}


void Skeleton::setJointRotation(unsigned index, Vec3f euler_angles) {
	Joint& joint = joints_[index];

	// For convenient reading, we store the rotation angles in the joint
	// as-is, in addition to setting the to_parent matrix to match the rotation.
	joint.rotation = euler_angles;

	joint.to_parent = Mat4f();
	joint.to_parent.setCol(3, Vec4f(joint.position, 1.0f));

	// YOUR CODE HERE (R2)
	// Modify the "to_parent" matrix of the joint to match
	// the given rotation Euler angles. Thanks to the line above,
	// "to_parent" already contains the correct transformation
	// component in the last column. Compute the rotation that
	// corresponds to the current Euler angles and replace the
	// upper 3x3 block of "to_parent" with the result.
	// Hints: You can use Mat3f::rotation() three times in a row,
	// once for each main axis, and multiply the results.

}

void Skeleton::incrJointRotation(unsigned index, Vec3f euler_angles) {
	setJointRotation(index, getJointRotation(index) + euler_angles);
}

void Skeleton::updateToWorldTransforms() {
	// Here we just initiate the hierarchical transformation from the root node (at index 0)
	// and an identity transformation, precisely as in the lecture slides on hierarchical modeling.
	updateToWorldTransforms(0, Mat4f());
}

void Skeleton::updateToWorldTransforms(unsigned joint_index, const Mat4f& parent_to_world) {
	// YOUR CODE HERE (R1)
	// Update transforms for joint at joint_index and its children.
}

void Skeleton::computeToBindTransforms() {
	updateToWorldTransforms();
	// YOUR CODE HERE (R4)
	// Given the current to_world transforms for each bone,
	// compute the inverse bind pose transformations (as per the lecture slides),
	// and store the results in the member to_bind_joint of each joint.
}

vector<Mat4f> Skeleton::getToWorldTransforms() {
	updateToWorldTransforms();
	vector<Mat4f> transforms;
	for (const auto& j : joints_)
		transforms.push_back(j.to_world);
	return transforms;
}

vector<Mat4f> Skeleton::getSSDTransforms() {
	updateToWorldTransforms();
	// YOUR CODE HERE (R4)
	// Compute the relative transformations between the bind pose and current pose,
	// store the results in the vector "transforms". These are the transformations
	// passed into the actual skinning code. (In the lecture slides' terms,
	// these are the T_i * inv(B_i) matrices.)
	// This initializes transforms with JOINTS amount of elements, so use the []-operator
	// to assign values into transforms, not push_back
	vector<Mat4f> transforms(JOINTS);
	return transforms;
}
