/*!
 *
 * \author VaHiD AzIzI
 *
 */


#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{

}

void getShapeCenter(Util::Vector& c, const std::vector<Util::Vector>& _shape)
{
	c.x = 0; 
	c.y = 0;
	c.z = 0;  
	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); ++it)
	{
		c.x += it->x; 
		c.y += it->y; 
		c.z += it->z; 
	}
	if(!_shape.empty())
	{
		c.x /= _shape.size(); 
		c.y /= _shape.size(); 
		c.z /= _shape.size(); 
	}
	return; 
}

bool detectConvex(const std::vector<Util::Vector>& _shape) {
	if (_shape.size() <= 2) {
		std::cerr << "not enough vertices in shape!!!" << std::endl;
		return false;
	}

	for(int i = 0; i < _shape.size(); ++i) {
		int next = i + 1;
		if (next == _shape.size()) {
			next = 0;
		}
		Util::Vector v1 = _shape[i];
		Util::Vector v2 = _shape[next];
		Util::Vector edge12 = v2 - v1;
		std::vector<Util::Vector> x_multiply_list;
		for (int j = 0; j < _shape.size(); ++j) {
			if (j == i || j == next) continue;
			Util::Vector v3 = _shape[j];
			Util::Vector edge13 = v3 - v1;
			Util::Vector x_multiply = Util::cross(edge12, edge13);
			x_multiply_list.push_back(x_multiply);
		}

		for (int j = 1; j < x_multiply_list.size(); ++j) {
			if (x_multiply_list[0]*x_multiply_list[j] < 0) {
	//			std::cout << "It's not a convex!!!" << std::endl;
				return false;
			}
		}
		x_multiply_list.clear();
	}
	return true;		
}

Util::Vector supportStep(const std::vector<Util::Vector>& _shape, Util::Vector direction)
{
	float dotprod = -100000000000; 
	float dotprod_t = 0; 
	std::vector<Util::Vector>::const_iterator it_t = _shape.begin(); 

	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); ++it)
	{
		dotprod_t = it->x * direction.x + it->y * direction.y + it->z * direction.z; 
		if(dotprod_t > dotprod)
		{
			dotprod = dotprod_t; 
			it_t = it; 
		}
	}
	Util::Vector returnVector(it_t->x, it_t->y, it_t->z); 
	return returnVector; 

}

Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector direction)
{
	Util::Vector A_point; 
	A_point = supportStep(_shapeA, direction); 

	direction *= -1;    // reverse the direction 
	//std::cout << direction.x << " "<< direction.y << " "<< direction.z << std::endl; 
	Util::Vector B_point; 
	B_point = supportStep(_shapeB, direction); 

	//std::cout << A_point.x << " "<< A_point.y << " "<< A_point.z << std::endl; 
	//std::cout << B_point.x << " "<< B_point.y << " "<< B_point.z << std::endl; 

	return (A_point - B_point); // shape A - shape B 
}

Util::Vector getDirection(const std::vector<Util::Vector>& simplexList)
{
	if(simplexList.size() < 2)
	{
		std::cout << "Error: getting direction"<< std::endl; 
		return Util::Vector(0, 0, 0); 
	}
	std::vector<Util::Vector>::const_reverse_iterator rit = simplexList.rbegin();
	Util::Vector lastSimplex(rit->x, rit->y, rit->z); 
	rit++; 
	Util::Vector secondLastSimplex(rit->x, rit->y, rit->z); 

	Util::Vector AB = lastSimplex - secondLastSimplex; 
	Util::Vector AO = lastSimplex * (-1);
	Util::Vector direction = AO*(AB*AB) - AB*(AB*AO); // (AB X AO) X AB;  

	return direction; 
}

/*bool cross(Util::Vector p1, Util::Vector p2, Util::Vector p3)
  {	
  if((p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y) > 0.0f)
  return true; 
  else
  return false; 
  }*/
bool onSegment(Util::Vector A, Util::Vector B, Util::Vector Origin)
{
	Util::Vector AB = B - A; 
	Util::Vector AO = -1 * A; 
	float rx = 0 , ry = 0, rz = 0; 
	bool x_e = false; 
	bool y_e = false; 
	bool z_e = false; // exclude
	if(AB.x == 0 || AO.x == 0 )
	{
		if(!(AB.x == 0 && AO.x == 0))
		{
			return false;  // not on the Segment
		}
		x_e = true; 
	}else
	{
		rx = AO.x/AB.x;
	}

	if(AB.y == 0 || AO.y == 0 )
	{
		if(!(AB.y == 0 && AO.y == 0))
		{
			return false;  // not on the Segment
		}
		y_e = true; 
	}else
	{
		ry = AO.y/AB.y;
	}

	if(AB.z == 0 || AO.z == 0 )
	{
		if(!(AB.z == 0 && AO.z == 0))
		{
			return false;  // not on the Segment
		}
		z_e = true; 
	}else
	{
		rz = AO.z/AB.z;
	}
	std::vector<float> r; 
	if(!x_e) r.push_back(rx);
	if(!y_e) r.push_back(ry);
	if(!z_e) r.push_back(rz);   

	if(r.size() == 3)
	{
		if(r[0] == r[1] && r[1] == r[2])
		{
			if(0 <= r[0] && r[0] <= 1)
			{
				return true; 
			}

		}else return false; 
	}
	else if(r.size() == 2)
	{
		if(r[0] == r[1])
		{
			if(0 <= r[0] && r[0] <= 1)
			{
				return true; 
			}

		}else return false; 

	}
	else if(r.size() == 1)
	{
		if(0 <= r[0] && r[0] <= 1)
		{
			return true; 
		}else return false; 
	}
	else return true; 
}


bool containsOrigin(std::vector<Util::Vector>& simplexList,  Util::Vector Origin)
{
	if(simplexList.size()!=3)
	{
		std::cout << "Error: check containsOrigin" << std::endl; 
		return false; 
	}

	Util::Vector C = simplexList[0]; 
	Util::Vector B = simplexList[1]; // the second last added simplex
	Util::Vector A = simplexList[2]; // the last added simplex

	Util::Vector AB = B - A; 
	Util::Vector AC = C - A; 
	Util::Vector AO = A * (-1);
	Util::Vector prep_AB = AC*(AB*AB) - AB*(AB*AC); // (AB X AC) X AB;  
	Util::Vector prep_AC = AB*(AC*AC) - AC*(AC*AB); // (AC X AB) X AC;  

	if(onSegment(A,  B, Origin) || onSegment(A,  C, Origin))
	{
		return true; 
	}

	if(prep_AB * AO <= 0)
	{// in the R4 area, remove C
		simplexList.erase(simplexList.begin());  
		return false; 
	}
	else if(prep_AC * AO <= 0)
	{// in the R3 area, remove b
		simplexList.erase(simplexList.begin() + 1); 
		return false;  
	}
	return true; 

}
void printShape(const std::vector<Util::Vector>&  _shape)
{
	std::cout<< "\n printing shape\n"; 
	for (std::vector<Util::Vector>::const_iterator it = _shape.begin(); it != _shape.end(); ++it)
	{
		std::cout << it->x << " " << it->y << " "<< it->z << std::endl; 
	}
}

void printVector(std::string s, Util::Vector v) {
	std::cout << "Vector " << s << " is " << v.x << " " << v.y << " " << v.z << std::endl;
}
bool getShortestEdge(const std::vector<Util::Vector>& simplexList, SteerLib::Edge& shortestEdge)
{
	float shortestDistance = FLT_MAX;

	for (int i = 0; i < simplexList.size(); ++i)
	{
		int next = i + 1;
		if (next == simplexList.size()) {
			next = 0;
		}

		Util::Vector AB = simplexList[next] - simplexList[i];
		Util::Vector AO = -1 * simplexList[i];
		Util::Vector perp;
		//check if A, B, O are on same line
		if (fabs(fabs(AB*AO) - AB.norm() * AO.norm()) < TOLERANCE) {
			perp.x = -1 * AB.z;
			perp.y = 0;
			perp.z = AB.x;
		} else {
			perp = AO*(AB*AB) - AB*(AB*AO);
		}

		//printVector("perp", perp);
		//printVector("AB", AB);
		perp = Util::normalize(perp);
		float distance = fabs(AO*perp);
		if (shortestDistance > distance) {
			shortestEdge.p1 = simplexList[i];
			shortestEdge.p2 = simplexList[next];
			shortestEdge.perp = -1 * perp;
			shortestEdge.index = next;
			shortestEdge.distance = distance;
			shortestDistance = distance;
		}
	}

	if (shortestDistance == FLT_MAX) {
		return false;
	} else {
		return true;
	}

}

bool EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> simplexList)
{
	while (1) {
		SteerLib::Edge shortestEdge;
		if (!getShortestEdge(simplexList, shortestEdge)) std::cerr << "No shortestEdge found!!!" << std::endl;

		Util::Vector simplex = support(_shapeA, _shapeB, shortestEdge.perp);
		float difference = fabs(simplex * shortestEdge.perp - shortestEdge.distance);
		if (difference < TOLERANCE) {
			return_penetration_depth = shortestEdge.distance;
			return_penetration_vector = shortestEdge.perp;
			return true;
		} else {
			simplexList.insert(simplexList.begin() + shortestEdge.index, simplex);
		}
	}
	return false;
}


bool detectClockwise(const std::vector<Util::Vector>& _shape) {
	int rightMostIndex = -1;
	float rightMostX = FLT_MIN;
	for (int i = 0; i < _shape.size(); ++i) {
		if (_shape[i].x > rightMostX) {
			rightMostIndex = i;
			rightMostX = _shape[i].x;
		}
	}

	if (rightMostX == FLT_MIN) {
		std::cerr << "Can't found right most vertex in shape!!!" << std::endl;
	}

	int pre = (rightMostIndex - 1) % _shape.size();
	int next = (rightMostIndex + 1) % _shape.size();
	Util::Vector pre_X = _shape[rightMostIndex] - _shape[pre];
	Util::Vector X_next = _shape[next] - _shape[rightMostIndex];
	Util::Vector cross_product = Util::cross(pre_X, X_next);

	if (cross_product.y < 0) {
		return true;
	} else if (cross_product.y > 0) {
		return false;
	} else {
		std::cerr << "detect clockwise failed!!" << std::endl;
		return true;
	}
}

std::vector<int>  getConcaveVertices(const std::vector<Util::Vector>& _shape, bool clockwise) {
	std::vector<int> concaveVerticesIdx;
	//std::cout << "need to implement getConcaveVertices!!!" << std::endl;
	for (int i = 0; i < _shape.size(); ++i) {

		int pre = (i - 1) % _shape.size();
		int next = (i + 1) % _shape.size();
		Util::Vector pre_X = _shape[i] - _shape[pre];
		Util::Vector X_next = _shape[next] - _shape[i];
		Util::Vector cross_product = Util::cross(pre_X, X_next);
		if (clockwise) {
			if (cross_product.y > 0) {
				concaveVerticesIdx.push_back(i);
			}
		} else {
			if (cross_product.y < 0) {
				concaveVerticesIdx.push_back(i);
			}
		}
	}
	return concaveVerticesIdx;
}

int getPre(int curr, std::vector<Util::Vector>& _shape, std::vector<int>& deletedVertices) {
	do {
		curr = (curr - 1) % _shape.size();
	} while (std::find(deletedVertices.begin(),deletedVertices.end(), curr) != deletedVertices.end());
	return curr;
}

int getNext(int curr, std::vector<Util::Vector>& _shape, std::vector<int>& deletedVertices) {
	do {
		curr = (curr + 1) % _shape.size();
	} while (std::find(deletedVertices.begin(),deletedVertices.end(), curr) != deletedVertices.end());
	return curr;
}
std::vector< std::vector<Util::Vector>> decompositeShape(const std::vector<Util::Vector>& _shape, std::vector<int>& concaveVertices, bool clockwise) {
	std::vector< std::vector<Util::Vector>> decompositedShapeList;
	std::vector< Util::Vector> temp_shape(_shape);
	std::vector< int> deletedVertices;
//	std::cout << "Not decomposited!!!" << std::endl;
	while (!detectConvex(temp_shape)) {
		for (int i = 0; i < concaveVertices.size(); ++i) {
			int current = concaveVertices[i];
	//		if (deletedVertices.find(current) != deletedVertices.end()) {
	//			std::cerr << "Error: concave vertex deleted!!!" << std::endl;
	//		}
			int pre = getPre(current, temp_shape, deletedVertices);
			int next = getNext(current, temp_shape, deletedVertices);
			//if next vertex is not concave
			if (std::find(concaveVertices.begin(),concaveVertices.end(), next) == concaveVertices.end()) {
				std::vector<Util::Vector> decompositedShape;
				decompositedShape.push_back(temp_shape[current]);
				decompositedShape.push_back(temp_shape[next]);
				deletedVertices.push_back(next);
				do {
					next = getNext(next, temp_shape, deletedVertices);
					decompositedShape.push_back(temp_shape[next]);
					deletedVertices.push_back(next);
				} while (detectConvex(decompositedShape));

				decompositedShape.pop_back();
				deletedVertices.pop_back();		
				deletedVertices.pop_back();		

				decompositedShapeList.push_back(decompositedShape);	
				continue;
			}

			// if previous vertex is not concave	
			if (std::find(concaveVertices.begin(),concaveVertices.end(), pre) == concaveVertices.end()) {
				std::vector<Util::Vector> decompositedShape;
				decompositedShape.push_back(temp_shape[current]);
				decompositedShape.push_back(temp_shape[pre]);
				deletedVertices.push_back(pre);
				do {
					pre = getPre(next, temp_shape, deletedVertices);
					decompositedShape.push_back(temp_shape[pre]);
					deletedVertices.push_back(pre);
				} while (detectConvex(decompositedShape));

				decompositedShape.pop_back();
				deletedVertices.pop_back();		
				deletedVertices.pop_back();		

				decompositedShapeList.push_back(decompositedShape);	
				continue;
			}
		}
		std::sort(deletedVertices.begin(), deletedVertices.end());
		for (int i = deletedVertices.size() - 1; i >= 0; --i) {
			temp_shape.erase(temp_shape.begin() + deletedVertices[i]);
		}
		deletedVertices.clear();
		concaveVertices.clear();
		concaveVertices = getConcaveVertices(temp_shape, clockwise);
	}

	decompositedShapeList.push_back(temp_shape);

	return decompositedShapeList;
}

std::vector< std::vector<Util::Vector> > decompositeShape(const std::vector<Util::Vector>& _shape) {

	bool clockwise = detectClockwise(_shape);
	std::vector<int> concaveVertices = getConcaveVertices(_shape, clockwise);
	std::vector< std::vector<Util::Vector> > decompositedShapeList = decompositeShape(_shape, concaveVertices, clockwise);
	return decompositedShapeList;
}

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
/********** for extra credit *************/

	if(!detectConvex(_shapeA) || !detectConvex(_shapeB)) {
		std::cerr << "shape A or B is not a convex!!" << std::endl; 
		std::vector<std::vector<Util::Vector>> decomp_A = decompositeShape(_shapeA);
		std::vector<std::vector<Util::Vector>> decomp_B = decompositeShape(_shapeB);
		
		for (int i = 0; i < decomp_A.size(); ++i) {
//			printShape(decomp_A[i]);
			for (int j = 0; j < decomp_B.size(); ++j) {
//				printShape(decomp_B[i]);
				if (intersect(return_penetration_depth, return_penetration_vector, decomp_A[i], decomp_B[j])) {
					return true;
				}
			}
		} 
		return false;
	}

/*********** until this line ***************/

	Util::Vector Origin(0,0,0);
	std::vector<Util::Vector> simplexList; 
	// compute the shape Center
	Util::Vector cA(0,0,0); 
	Util::Vector cB(0,0,0); 
	getShapeCenter(cA, _shapeA); 
	getShapeCenter(cB, _shapeB); 

	Util::Vector direction = cA - cB; 
	if(direction  == Origin)
	{
		return true; 
	}
	//std::cout << direction.x << " "<< direction.y << " "<< direction.z << std::endl; 
	Util::Vector simplex = support(_shapeA, _shapeB, direction); 
	simplexList.push_back(simplex); 
//	std::cout <<"Simplex: "  << simplex.x << " "<< simplex.y << " "<< simplex.z << std::endl; 

	direction *= -1; // reverse direction
	simplex = support(_shapeA, _shapeB, direction); 
	simplexList.push_back(simplex);        // get the first two simplex. build a line
//	std::cout <<"Simplex: "  << simplex.x << " "<< simplex.y << " "<< simplex.z << std::endl; 

	int index = 0; 
//	std::cout <<"Index= " << index <<" " << direction.x << " "<< direction.y << " "<< direction.z << std::endl; 
	while(true) // temp 4 times
	{
		index++; 
		direction = getDirection(simplexList); 

		if(direction == Origin) // if direction = 0,0,0 // origin is on the line AB
		{
			//std::cout << " direction == Origin" << std::endl;
			if(simplexList.size() != 2){
				std::cout << "Error: Direction Origin " << std::endl; 
				break; 
			}

			std::vector<Util::Vector>::iterator it = simplexList.begin(); 
			Util::Vector p1(it->x, it->y, it->z);
			it++;  
			Util::Vector p2(it->x, it->y, it->z);
			if(onSegment(p1, p2, Origin))
			{
				EPA(return_penetration_depth, return_penetration_vector,_shapeA , _shapeB, simplexList); 
				return true; 
			}
			else
				return false; 
		}

		simplex = support(_shapeA, _shapeB, direction);   

//		std::cout <<"Index= " << index <<" " << direction.x << " "<< direction.y << " "<< direction.z << std::endl; 

		simplexList.push_back(simplex); 
//		std::cout <<"Simplex: "  << simplex.x << " "<< simplex.y << " "<< simplex.z << std::endl; 

		if(simplex * direction <= 0){
			//the new simplex is not past the origin in the direction
			//impossibly contain the origin 
			return false; 
		} else {
			if(containsOrigin(simplexList, Origin))
			{
				EPA(return_penetration_depth, return_penetration_vector,_shapeA , _shapeB, simplexList); 
				return true; 
			}
		}

	}




	return false; // There is no collision
}


