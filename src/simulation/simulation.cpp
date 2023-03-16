#include "simulation.hpp"

using namespace cgp;


void collision_between_box_and_floor(box_structure& box, vec3 point, vec3 floor_normal, float dt) {
	//Handle collision between floor and cylinder
	// 
	//Find the angles defining the orientation of the cylinder
	vec3 p0 = box.rot*box.vertices[0];
	vec3 p1 = box.rot*box.vertices[1];
	float cos_phi = dot(vec3{ p1.x - p0.x, p1.y - p0.y, 0 }, vec3{ 1,0,0 }) / norm(vec3{ p1.x - p0.x, p1.y - p0.y, 0 });
	float sin_phi = dot(vec3{ p1.x - p0.x, p1.y - p0.y, 0 }, vec3{ 0,-1,0 }) / norm(vec3{ p1.x - p0.x, p1.y - p0.y, 0 });

	float theta = std::acos(dot(p1-p0, floor_normal)/norm(p1-p0)); //angle with rotation around y-axis
	float phi = std::atan(sin_phi / cos_phi); //angle with z-axis
	if (cos_phi < 0) {
		phi += Pi;
	}

	//Find contact point
	vec3 vec_orth = box.rot * rotation_transform::from_axis_angle({ -1,0,0 }, Pi / 2) * (box.vertices[1] - box.vertices[0]);
	float dist_sol = std::abs((box.box_hauteur / 2)* std::cos(theta)) + std::abs(box.r*std::sin(theta));
	if (std::abs(dot(box.p - point, floor_normal)) <= dist_sol) {
		float l_z = (box.box_hauteur / 2) * sin(theta) - box.r * cos(theta); //distance z-axis/point_contact
		vec3 p_contact = l_z* vec3{ -cos(phi), sin(phi),0 } + vec3{ box.p.x, box.p.y, 0 };
		if (p1.z < p0.z) {
			p_contact = l_z * vec3{ cos(phi), -sin(phi),0 } + vec3{ box.p.x, box.p.y, 0 };
		}

		box.p_contact_floor = p_contact;
		
		float alpha = 0.8f;
		float beta = 0.6f;
		
		//Update speed and angular momentum
		vec3 v_orth = dot(box.v, floor_normal) * floor_normal;
		vec3 v_par = box.v - v_orth;
		box.v = alpha * v_par - beta * v_orth;

		if (!isnan(phi)) {
			vec3 r1 = p_contact - box.p;
			vec3 v_w = { 0,0,0 }; 
			if (norm(box.w) < 100.0f) {
				v_w = (norm(box.w) * box.box_hauteur / 2) * floor_normal;
				box.L += cross(r1, -(v_orth - 1.9f * v_w));
			}

			if (box.p.z < box.r + 0.1f) {
				box.L *= 0.1f;
			}
			else {
				box.L *= 0.9f;
			}
		}

		//Offset to avoid penetration
		float d = dist_sol - dot(box.p - point, floor_normal);
		box.p = box.p + d * floor_normal;
	}
}
void collision_between_box_and_particle(particle_structure& particle, box_structure& box, float dt) {
	//Handle collision between cylinder and sphere

	//Some parameters
	float epsilon = 0.0001f;
	float alpha = 0.9f;
	float beta = 0.5f;
	float mu = 0.5f;

	//Check if the sphere is near enough for collision
	vec3 p1 = box.rot * box.vertices[1] + box.p; //Global position of top vertex
	vec3 p0 = box.rot * box.vertices[0] + box.p; //Global position of bottom vertex
	vec3 vec_par = dot(particle.p - p1, normalize(p0 - p1)) * normalize(p0 - p1);
	vec3 vec_orth = particle.p - p1 - vec_par;
	if ((norm(box.p - particle.p) < box.box_hauteur/2 + particle.r) && (norm( vec_orth) <= particle.r + box.r) ) {
		//std::cout << "Collision sphere-cylinder detected" << std::endl;
		vec3 p_contact = particle.p - particle.r*normalize(vec_orth);	
		box.p_contact_ball = p_contact;
		vec3 normal = normalize(vec_orth);

		//Update speed and angular momentum
		vec3 r1 = p_contact - box.p;
		vec3 r2 = p_contact - particle.p;
		vec3 v1 = box.v + cross(box.w, box.rot * r1);
		vec3 v2 = particle.v + cross(particle.w, particle.rot * r2);
		vec3 inertie = cross(inverse(box.inertie) * cross(r1, normal), r1) + cross(inverse(particle.inertie) * cross(r2, normal), r2);
		float K = 1 / box.m + 1 / particle.m + dot(normal, inertie);
		float j_n = dot(v1 - v2, normal) / K;
		vec3 J = j_n * normal;

		box.v -= J / box.m;
		particle.v += J / particle.m;
		box.L -= cross(r1, J);
		particle.L += cross(r2, J);

		//Offset to avoid penetration
		float d = particle.r + box.r - norm(vec_orth);
		box.p -= d * normal;
	}
}
void collision_between_boxes(box_structure& box1, box_structure& box2) {
	//APPROACH 1 : calculate the intersecting points
	//Project the top vertex of each cylinder and check whether the projection is inside the cylinder or not
	vec3 p0_a = box1.rot * box1.vertices[0] + box1.p; //Bottom point of cylinder A
	vec3 p1_a = box1.rot * box1.vertices[1] + box1.p; //Top point of cylinder A
	vec3 p0_b = box2.rot * box2.vertices[0] + box2.p; //Bottom point of cylinder B
	vec3 p1_b = box2.rot * box2.vertices[1] + box2.p; //Top point of cylinder B
	vec3 s1 = p1_a - p0_a; //directionnal vector of cylinder A
	vec3 s2 = p1_b - p0_b; //directionnal vector of cylinder B

	float alpha = dot(p1_a - p0_b, s2)/(norm(s2)*norm(s2));
	vec3 p1_a_clamp = p0_b + alpha * s2;

	//If the projection is below the cylinder, we set it to the bottom vertex
	if (alpha < 0) {
		p1_a_clamp = p0_b;
	}

	//If the projection is above the cylinder, we set it to the top vertex
	if (alpha > 1) {
		p1_a_clamp = p1_b;
	}
	
	float beta = dot(p1_b - p0_a, s1)/(norm(s1)*norm(s1));
	vec3 p1_b_clamp = p0_a + beta * s1;
	//If the projection is below the cylinder, we set it to the bottom vertex
	if (beta < 0) {
		p1_b_clamp = p0_a;
	}
	//If the projection is above the cylinder, we set it to the top vertex
	if (beta >1 ) {
		p1_b_clamp = p1_a;
	}

	vec3 p_contact = (p1_a_clamp + p1_b_clamp) / 2; // Contact point is the mean of the projection on each cylinder
	box1.p_contact_cylinder = p_contact;
	box2.p_contact_cylinder = p_contact;

	if (norm(p1_a_clamp - p1_b_clamp) < box1.r + box2.r) {
		//std::cout << "Collision between cylinders detected" << std::endl;

		//Update speed and angular momentum
		vec3 normal = normalize(p1_b_clamp - p1_a_clamp);
		vec3 r1 = p_contact - box1.p;
		vec3 r2 = p_contact - box2.p;
		vec3 v1 = box1.v + cross(box1.w, box1.rot * r1);
		vec3 v2 = box2.v + cross(box2.w, box2.rot * r2);
		vec3 inertie = cross(inverse(box1.inertie) * cross(r1, normal), r1) + cross(inverse(box2.inertie) * cross(r2, normal), r2);
		float K = 1 / box1.m + 1 / box2.m + dot(normal, inertie);
		float j_n = dot(v1 - v2, normal) / K;
		vec3 J = j_n * normal;
		
		box1.v -= J / box1.m;
		box2.v += J / box2.m;
		box1.L -= 0.7f*cross(r1, J);
		box2.L += 0.7f*cross(r2, J);
		
		//Offset to avoid penetration
		float d = box1.r + box2.r - norm(p1_a_clamp - p1_b_clamp);
		box1.p += d * normal;
		box2.p -= d * normal;
	}
}
void plane_collision(particle_structure& particle, vec3 point, vec3 normal, float alpha, float beta) {
	//Handle ball collision with the floor
	if (dot(particle.p - point, normal) <= particle.r) {
		vec3 v_orth = dot(particle.v, normal) * normal;
		vec3 v_par = particle.v - v_orth;
		particle.v = alpha * v_par - beta * v_orth;

		float d = particle.r - dot(particle.p - point, normal);
		particle.p = particle.p + d * normal;
	}
}
void simulate(particle_structure& particle, std::vector<box_structure>& boxes, float dt)
{
	//Some constant values
	vec3 const g = 2*vec3{ 0,0,-9.81f };
	float alpha = 1.0f;
	float beta = 0.9f;


	//Defining points and normal vectors for each face of the cube
	vec3 bottom_point = vec3(1.0, 1.0, 0.0);
	vec3 bottom_normal = vec3(0.0, 0.0, 1.0);


	//Compute particle evolution
	mat3 I;
	quaternion qw;
	quaternion q_rot;
	vec3 const f_particle = particle.m * g;
	I = particle.rot.matrix() * particle.inertie * transpose(particle.rot.matrix());
	particle.w = inverse(I) * particle.L;

	particle.v = (1 - 0.9f * dt) * particle.v + dt * f_particle / particle.m;
	particle.p = particle.p + dt * particle.v;

	qw = quaternion(particle.w, 0);
	q_rot = particle.rot.data + dt * 0.5f * qw * particle.rot.data;
	particle.rot = rotation_transform::from_quaternion(normalize(q_rot));

	//Compute boxes evolution 
	size_t N = boxes.size();
	for (int k = 0; k < N; k++) {
		box_structure& box = boxes[k];

		//Compute net force F and torque
		vec3 f_box = { 0,0,0 };
		vec3 torque = { 0,0,0 };
		for (int i = 0; i < box.vertices.size(); i++) {
			f_box += (box.m / box.vertices.size()) * g;
		}

		I = box.rot.matrix() * box.inertie * transpose(box.rot.matrix());
		box.w = inverse(I) * box.L;

		box.v =  box.v + dt * f_box / box.m;
		box.p += dt * box.v;

		qw = quaternion(box.w, 0);
		q_rot = box.rot.data + dt * 0.5f * qw * box.rot.data;
		box.rot = rotation_transform::from_quaternion(normalize(q_rot));
		box.L += dt * torque;

		//Handle cylinder collision
		collision_between_box_and_particle(particle, box, dt);
		collision_between_box_and_floor(box, bottom_point, bottom_normal, dt);

		for (int i = 0; i < N; i++) {
			if (i != k) {
				box_structure& other_box = boxes[i];
				collision_between_boxes(box, other_box);
			}
		}
	}
	

	//Handle sphere collision with floor
	plane_collision(particle, bottom_point, bottom_normal, alpha, beta);
}
