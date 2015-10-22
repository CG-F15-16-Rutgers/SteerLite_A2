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



Util::Vector support(const std::vector<Util::Vector>& _shape, Util::Vector direction)
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

//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	std::vector<Util::Vector> shapeA;
	Util::Vector tmp1(4,0,11); 
	Util::Vector tmp2(4,0,5); 
	Util::Vector tmp3(9,0,9); 

	shapeA.push_back(tmp1); 
	shapeA.push_back(tmp2); 
	shapeA.push_back(tmp3); 
    
    std::vector<Util::Vector> shapeB;
	Util::Vector tmp4(5,0,7); 
	Util::Vector tmp5(12,0,7); 
	Util::Vector tmp6(7,0,3); 
	Util::Vector tmp7(10,0,2); 

	shapeB.push_back(tmp4); 
	shapeB.push_back(tmp5); 
	shapeB.push_back(tmp6); 
	shapeB.push_back(tmp7); 
/////////////////////////////////////////////////////////////
	// compute the shape Center
	Util::Vector cA(0,0,0); 
	Util::Vector cB(0,0,0); 
	getShapeCenter(cA, shapeA); 
	getShapeCenter(cB, shapeB); 

	Util::Vector direction = cA - cB; 
	std::cout << direction.x << " "<< direction.y << " "<< direction.z << std::endl; 
	
	Util::Vector A_point; 
	A_point = support(shapeA, direction); 
	Util::Vector B_point; 
	B_point = support(shapeB, direction); 

	std::cout << A_point.x << " "<< A_point.y << " "<< A_point.z << std::endl; 
	std::cout << B_point.x << " "<< B_point.y << " "<< B_point.z << std::endl; 




    return false; // There is no collision
}
