#pragma once

#include "cgp/cgp.hpp"


struct particle_structure
{
    float r;     // Radius_longueur
    float m;     // Mass
    cgp::vec3 p; // Position
    cgp::vec3 v; // Speed
    cgp::vec3 w; // Angular speed
    cgp::rotation_transform rot; // Rotation data

    cgp::vec3 c; // Color

    cgp::mat3 inertie ; // Inertia matrix
    cgp::vec3 L; // Angular momentum

};

struct box_structure
{
    float r; // Radius
    float box_hauteur; // Height
    float m;// Mass

    cgp::vec3 p;// Position
    cgp::vec3 v;// Speed
    cgp::vec3 w;// Angular speed
    cgp::vec3 p_contact_floor;// Contact point with floor, for display and debugging purpose
    cgp::vec3 p_contact_ball;// Contact point with ball, for display and debugging purpose
    cgp::vec3 p_contact_cylinder;// Contact point with cylinder, for display and debugging purpose
    cgp::rotation_transform rot; // Rotation data
    cgp::mat3 inertie; // Inertia matrix
    std::vector<cgp::vec3> vertices; // Bottom and top vertices of the cylinder
    cgp::vec3 L; // Angular momentum
    cgp::vec3 c;// Color
};

void collision_between_box_and_floor(box_structure& box, cgp::vec3 point, cgp::vec3 floor_normal, float dt);
void collision_between_box_and_particle(particle_structure& particle, box_structure& box, float dt);
void collision_between_boxes(box_structure& box1, box_structure& box2);
void plane_collision(particle_structure& particle, cgp::vec3 point, cgp::vec3 normal, float alpha, float beta);
void simulate(particle_structure& particle, std::vector<box_structure>& boxes, float dt);


