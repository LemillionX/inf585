#pragma once

#include "cgp/cgp.hpp"

#include "simulation/simulation.hpp"




// The element of the GUI that are not already stored in other structures
struct gui_parameters {
	bool display_frame = false;
	bool display_floor_contact_points = false;
	bool display_ball_contact_points = false;
	float ball_speed = 70.0f;
};



// The structure of the custom scene
struct scene_structure {
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //

	cgp::mesh_drawable global_frame;          // The standard global frame
	cgp::scene_environment_basic environment; // Standard environment controler
	gui_parameters gui;                       // Standard GUI element storage
	
	// ****************************** //
	// Elements and shapes of the scene
	// ****************************** //
	cgp::timer_event_periodic timer;
	particle_structure bowling_ball;
	std::vector<box_structure> boxes;
	cgp::mesh_drawable sphere;
	cgp::mesh_drawable floor;
	cgp::mesh_drawable bowling_pin;
	cgp::mesh_drawable point_contact_floor;
	cgp::mesh_drawable point_contact_ball;
	cgp::mesh_drawable point_contact_cylinder;

	bool ball_thrown = false; // Indicates if the ball was thrown by user
	float longueur_piste = 40.0f; // Floor length, for display purpose only
	float largeur_piste = 4.0f; // Floor width, for display purpose only
	float pin_r = 0.25f; // Pin's radius
	float pin_hauteur = 1.5f; // Pin's height

	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop

	void initialize_ball(); // Initalize the bowling ball
	void initialize_pins(); // Initialize the bowling pins
	void throw_ball(); //Throw the bowling ball

	void simulation_step(float dt);
	void sphere_display();
};





