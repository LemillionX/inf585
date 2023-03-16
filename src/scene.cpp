#include "scene.hpp"


using namespace cgp;


void scene_structure::display()
{
	timer.update();
	environment.light = environment.camera.position();

	// Call the simulation of the particle system
	float const dt = 0.01f * timer.scale;
	simulate(bowling_ball, boxes, dt);

	// Display the result
	sphere_display();


	if (gui.display_frame)
		draw(global_frame, environment);
}


void scene_structure::sphere_display()
{

	//Display the bowling pins
	size_t const N = boxes.size();
	for (size_t k = 0; k < N; ++k) {
		box_structure const& box = boxes[k];
		bowling_pin.shading.color = box.c;
		bowling_pin.transform.translation = box.p;
		bowling_pin.transform.rotation = box.rot;
		draw(bowling_pin, environment);

		//Display contact point
		if (gui.display_floor_contact_points) {
			point_contact_floor.shading.color = { 0,1,0 };
			point_contact_floor.transform.translation = box.p_contact_floor;
			point_contact_floor.transform.scaling = 0.1f;
			draw(point_contact_floor, environment);
		}

		if (gui.display_ball_contact_points) {
			point_contact_ball.shading.color = { 0,0,1 };
			point_contact_ball.transform.translation = box.p_contact_ball;
			point_contact_ball.transform.scaling = 0.1f;
			draw(point_contact_ball, environment);
		}

		/*
			point_contact_cylinder.shading.color = { 0,1,1 };
			point_contact_cylinder.transform.translation = box.p_contact_cylinder;
			point_contact_cylinder.transform.scaling = 0.1f;
			draw(point_contact_cylinder, environment);
		*/

	}

	//Display the bowling ball
	sphere.shading.color = bowling_ball.c;
	sphere.transform.translation = bowling_ball.p;
	sphere.transform.scaling = bowling_ball.r;
	sphere.transform.rotation = bowling_ball.rot;
	draw(sphere, environment);


	
	// Display the floor
	draw(floor, environment);

}

void scene_structure::initialize_ball() {
	bowling_ball.c = { 0,0,0 };
	bowling_ball.m = 7.0f;
	bowling_ball.r = 0.4f;
	bowling_ball.p = { 0,0,bowling_ball.r };
	bowling_ball.v = vec3(0.0f, 0.0f, 0.0f);
	bowling_ball.w = vec3(0.0f, 0.0f, 0.0f);
	bowling_ball.rot = rotation_transform();
	bowling_ball.inertie = (2.0f/5.0f)* bowling_ball.m* std::pow(bowling_ball.r,2.0f) *mat3{ 1.0f,0,0,
																							 0, 1.0f,0,
																							 0, 0, 1.0f };
	bowling_ball.L = bowling_ball.inertie * bowling_ball.w;
}

void scene_structure::initialize_pins() {
	boxes.clear();

	
	vec3 pin1_pos = { 5.0f, 0.0f, pin_hauteur / 2 };

	// Pins position depend on the position of first one, such that it looks like: 
	//		 7 8 9 10
	//		  4 5 6
	//		   2 3
	//			1
	std::vector<vec3> pins_positions = {
		pin1_pos, 
		pin1_pos + vec3{2 * pin_r, - pin_r, 0 }, pin1_pos + vec3{2 * pin_r, pin_r,  0 },
		pin1_pos + vec3{4 * pin_r, -2* pin_r, 0 }, pin1_pos + vec3{4 * pin_r, 0, 0 }, pin1_pos + vec3{4 * pin_r, 2 * pin_r, 0 },
		pin1_pos + vec3{6 * pin_r, -3 * pin_r, 0 }, pin1_pos + vec3{6 * pin_r, -pin_r, 0 }, pin1_pos + vec3{6 * pin_r,pin_r, 0 },pin1_pos + vec3{6 * pin_r, 3 * pin_r, 0 }
	};

	//Initialize each pin
	for (int i = 0; i < 10; i++) {
		box_structure box;
		box.box_hauteur = pin_hauteur;
		const vec3 c0 = { 0,0, -pin_hauteur/2 };
		const vec3 c1 = { 0,0, pin_hauteur/2 };
		box.vertices.push_back(c0);
		box.vertices.push_back(c1);
		box.rot = rotation_transform::from_axis_angle({ 0,0,1 }, Pi / 4) * rotation_transform::from_axis_angle({ 1,0,0 }, 0);
		box.r = pin_r;
		box.p = pins_positions[i];
		box.p_contact_floor = box.p;
		box.p_contact_ball = bowling_ball.p;
		box.p_contact_cylinder = box.p;
		box.v = { 0.0f, 0.0f, 0.0f };
		box.c = { 1.0f, 0.0f, 0.0f };
		box.m = 1.5f;
		box.w = { 0.0f,0,0 };
		box.inertie = box.m * mat3{ 0.75f * std::pow(box.r,2.0f) + 0.3f * std::pow(box.box_hauteur,2.0f),0,0,
										  0,0.75f * std::pow(box.r,2.0f) + 0.3f * std::pow(box.box_hauteur,2.0f),0,
											0,0,0.5f * std::pow(box.r,2.0f)
		};
		box.L = box.inertie * box.w;
		boxes.push_back(box);
	}


}

void scene_structure::throw_ball() {
	bowling_ball.v = { gui.ball_speed, 0.0f, 0.0f };
	bowling_ball.w = { 0.0f, 0.0f, 0.0f };
}


void scene_structure::initialize()
{
	// Common element of the scene
	// ************************************ //
	global_frame.initialize(mesh_primitive_frame(), "Frame");
	environment.camera.look_at({ -4.0f,0.0f,1.0f }, { 0,0,0 }, { 0,0,1 });
	timer.event_period = 0.5f;

	// Set up floor's vertices
	const vec3 p00 = { 0,-largeur_piste / 2,0 };
	const vec3 p10 = { longueur_piste,-largeur_piste/2,0 };
	const vec3 p01 = { 0,largeur_piste / 2,0 };
	const vec3 p11 = { longueur_piste,largeur_piste / 2,0 };

	// Set up cylinder's vertices
	const vec3 c0 = { 0,0,-pin_hauteur/2 }; //Bottom vertex of the cylinder
	const vec3 c1 = { 0,0, pin_hauteur/2}; //Top vertex of the cylinder

	sphere.initialize(mesh_primitive_sphere(), "Sphere");
	bowling_pin.initialize(mesh_primitive_cylinder(pin_r, c0, c1, 10, 20, true));
	point_contact_floor.initialize(mesh_primitive_sphere(), "Point de contact au sol"); //Setup cylinder's contact points with the floor  
	point_contact_ball.initialize(mesh_primitive_sphere(), "Point de contact avec balle"); //Setup cylinder's contact points with the ball
	point_contact_cylinder.initialize(mesh_primitive_sphere(), "Point de contact avec cylindre"); //Setup cylinder's contact points with other cylinder
	floor.initialize(mesh_primitive_quadrangle(p00, p10, p11, p01), "Floor"); //Setup the floor
	initialize_ball();// Set up the bowling ball
	initialize_pins();//Set up pins
}


void scene_structure::display_gui()
{
	ImGui::Checkbox("Frame", &gui.display_frame);
	ImGui::SliderFloat("Time scale", &timer.scale, 0.05f, 2.0f, "%.2f s");
	ImGui::Checkbox("Floor contact points", &gui.display_floor_contact_points);
	ImGui::Checkbox("Ball contact points", &gui.display_ball_contact_points);
	ImGui::SliderFloat("Ball speed", &gui.ball_speed, 10.0f, 100.0f);
	bool launch = false;
	bool reset = false;
	bool move_left = false;
	bool move_right = false;
	float step = 0.1f;

	move_left |= ImGui::Button("Move Ball to Left");
	move_right |= ImGui::Button("Move Ball to Right");
	ImGui::Spacing(); ImGui::Spacing();
	launch |= ImGui::Button("Launch");
	ImGui::Spacing(); ImGui::Spacing();
	reset |= ImGui::Button("Reset");



	if (launch && !ball_thrown) {
		throw_ball();
		ball_thrown = true;
	}

	if (reset) {
		initialize_ball();// Set up the bowling ball
		initialize_pins();//Set up pins
		ball_thrown = false;
	}

	if (move_left && !ball_thrown) {
		bowling_ball.p.y += step;
	}
	if (move_right && !ball_thrown) {
		bowling_ball.p.y -= step;
	}

}


