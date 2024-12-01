#include "raylib.h"
#include "raymath.h"
#include <vector>
#include <cmath>

// Screen dimensions
const int screenWidth = 1200;
const int screenHeight = 800;

// Number of boids and parameters for their behavior
const int NUM_BOIDS = 1000;
const float MAX_SPEED = 2.5f;            // Maximum speed of each boid
const float MAX_FORCE = 0.1f;            // Maximum force applied to a boid
const float NEIGHBOR_RADIUS = 100.0f;    // Radius to consider for neighbors (for alignment and cohesion)
const float SEPARATION_RADIUS = 20.0f;   // Radius to consider for separation behavior
const float COHESION_WEIGHT = 0.5f;      // Weight for the cohesion behavior
const float ALIGNMENT_WEIGHT = 0.1f;     // Weight for the alignment behavior
const float SEPARATION_WEIGHT = 0.2f;    // Weight for the separation behavior

// Boid structure, representing each individual boid
struct Boid {
    Vector2 position;       // Position of the boid
    Vector2 velocity;       // Velocity of the boid
    Vector2 acceleration;   // Acceleration (used for force calculations)
    float rotation;         // Rotation angle based on the boid's velocity

    // Constructor to initialize a boid with a given position
    Boid(Vector2 position) {
        this->position = position;
        velocity = { (float)GetRandomValue(-2, 2), (float)GetRandomValue(-2, 2) };  // Random initial velocity
        acceleration = { 0.0f, 0.0f };  // No initial acceleration
        rotation = atan2(velocity.y, velocity.x);  // Calculate the rotation based on the initial velocity
    }

    // Update the boid's position based on its velocity and acceleration
    void update() {
        velocity = Vector2Add(velocity, acceleration);  // Add acceleration to velocity
        velocity = Vector2Scale(Vector2Normalize(velocity), MAX_SPEED);  // Limit velocity to max speed
        position = Vector2Add(position, velocity);  // Update position based on velocity

        acceleration = { 0.0f, 0.0f };  // Reset acceleration after each update
        rotation = atan2(velocity.y, velocity.x);  // Update rotation based on the new velocity
    }

    // Apply a force to the boid by adding it to its acceleration
    void applyForce(Vector2 force) {
        acceleration = Vector2Add(acceleration, force);  // Add the force to the acceleration
    }

    // Ensure the boid wraps around the screen edges
    void borders() {
        if (position.x < 0) position.x = screenWidth;   // Wrap around horizontally
        if (position.x > screenWidth) position.x = 0;   // Wrap around horizontally
        if (position.y < 0) position.y = screenHeight;  // Wrap around vertically
        if (position.y > screenHeight) position.y = 0;  // Wrap around vertically
    }

    // Separation behavior: steer away from nearby boids to avoid crowding
    Vector2 separate(std::vector<Boid>& boids) {
        Vector2 steer = { 0.0f, 0.0f };  // Vector to store the steering force
        int count = 0;

        // Loop through all boids and check their distance to the current boid
        for (auto& other : boids) {
            float d = Vector2Distance(position, other.position);
            if (d > 0 && d < SEPARATION_RADIUS) {  // If the boid is within the separation radius
                Vector2 diff = Vector2Subtract(position, other.position);  // Vector pointing away from the other boid
                diff = Vector2Normalize(diff);  // Normalize the vector
                diff = Vector2Scale(diff, 1.0f / d);  // Scale by the inverse of the distance to add more force for closer boids
                steer = Vector2Add(steer, diff);  // Add this force to the steering vector
                count++;
            }
        }

        // If there are nearby boids, average the steering forces
        if (count > 0) {
            steer = Vector2Scale(steer, 1.0f / (float)count);  // Normalize the force by the number of boids
        }

        // If there is any steering force, limit it to the maximum force
        if (Vector2Length(steer) > 0) {
            steer = Vector2Normalize(steer);  // Normalize the steering force
            steer = Vector2Scale(steer, MAX_SPEED);  // Scale to maximum speed
            steer = Vector2Subtract(steer, velocity);  // Subtract the current velocity to get the required force
            if (Vector2Length(steer) > MAX_FORCE) {
                steer = Vector2Scale(Vector2Normalize(steer), MAX_FORCE);  // Limit the force to MAX_FORCE
            }
        }

        return steer;  // Return the separation steering force
    }

    // Alignment behavior: steer towards the average velocity of nearby boids
    Vector2 align(std::vector<Boid>& boids) {
        Vector2 sum = { 0.0f, 0.0f };  // Sum of the velocities of nearby boids
        int count = 0;

        // Loop through all boids and check their distance to the current boid
        for (auto& other : boids) {
            float d = Vector2Distance(position, other.position);
            if (d > 0 && d < NEIGHBOR_RADIUS) {  // If the boid is within the neighbor radius
                sum = Vector2Add(sum, other.velocity);  // Add the other boid's velocity to the sum
                count++;
            }
        }

        // If there are nearby boids, average the velocities
        if (count > 0) {
            sum = Vector2Scale(sum, 1.0f / (float)count);  // Average the velocities
            sum = Vector2Normalize(sum);  // Normalize the average velocity
            sum = Vector2Scale(sum, MAX_SPEED);  // Scale to the maximum speed
            Vector2 steer = Vector2Subtract(sum, velocity);  // Steer towards the average velocity
            if (Vector2Length(steer) > MAX_FORCE) {
                steer = Vector2Scale(Vector2Normalize(steer), MAX_FORCE);  // Limit the steering force
            }
            return steer;  // Return the alignment steering force
        }

        return { 0.0f, 0.0f };  // No steering force if no nearby boids
    }

    // Cohesion behavior: steer towards the average position of nearby boids
    Vector2 cohesion(std::vector<Boid>& boids) {
        Vector2 sum = { 0.0f, 0.0f };  // Sum of the positions of nearby boids
        int count = 0;

        // Loop through all boids and check their distance to the current boid
        for (auto& other : boids) {
            float d = Vector2Distance(position, other.position);
            if (d > 0 && d < NEIGHBOR_RADIUS) {  // If the boid is within the neighbor radius
                sum = Vector2Add(sum, other.position);  // Add the other boid's position to the sum
                count++;
            }
        }

        // If there are nearby boids, average the positions
        if (count > 0) {
            sum = Vector2Scale(sum, 1.0f / (float)count);  // Average the positions
            Vector2 steer = Vector2Subtract(sum, position);  // Steer towards the average position
            steer = Vector2Normalize(steer);  // Normalize the steering vector
            steer = Vector2Scale(steer, MAX_SPEED);  // Scale to maximum speed
            steer = Vector2Subtract(steer, velocity);  // Subtract the current velocity to get the required force
            if (Vector2Length(steer) > MAX_FORCE) {
                steer = Vector2Scale(Vector2Normalize(steer), MAX_FORCE);  // Limit the force to MAX_FORCE
            }
            return steer;  // Return the cohesion steering force
        }

        return { 0.0f, 0.0f };  // No steering force if no nearby boids
    }

    // Draw the boid on the screen as a triangle (representing the boid)
    void draw() {
        Vector2 front = { position.x + cos(rotation) * 10, position.y + sin(rotation) * 10 };  // Front of the boid
        Vector2 left = { position.x + cos(rotation + (float)PI / 3) * 6, position.y + sin(rotation + (float)PI / 3) * 6 };  // Left wing
        Vector2 right = { position.x + cos(rotation - (float)PI / 3) * 6, position.y + sin(rotation - (float)PI / 3) * 6 };  // Right wing

        // Draw the boid using three triangles (body + two wings)
        DrawTriangle(position, front, left, BLUE);
        DrawTriangle(position, front, right, BLUE);
    }
};

// Main program loop
int main() {
    InitWindow(screenWidth, screenHeight, "Boid Flocking Simulation");  // Initialize the window

    // Create a vector of boids with random initial positions
    std::vector<Boid> boids;
    for (int i = 0; i < NUM_BOIDS; i++) {
        boids.push_back(Boid({ (float)GetRandomValue(0, screenWidth), (float)GetRandomValue(0, screenHeight) }));
    }

    SetTargetFPS(60);  // Set the game to run at 60 frames per second

    // Main game loop
    while (!WindowShouldClose()) {
        // Apply the flocking behaviors (separation, alignment, cohesion) to each boid
        for (auto& boid : boids) {
            Vector2 sep = boid.separate(boids);  // Separation force
            Vector2 ali = boid.align(boids);     // Alignment force
            Vector2 coh = boid.cohesion(boids);  // Cohesion force

            // Scale the forces by their respective weights
            sep = Vector2Scale(sep, SEPARATION_WEIGHT);
            ali = Vector2Scale(ali, ALIGNMENT_WEIGHT);
            coh = Vector2Scale(coh, COHESION_WEIGHT);

            // Apply the forces to the boid
            boid.applyForce(sep);
            boid.applyForce(ali);
            boid.applyForce(coh);

            boid.update();    // Update boid's position and velocity
            boid.borders();   // Ensure boid wraps around the screen edges
        }

        // Drawing section
        BeginDrawing();
        ClearBackground(RAYWHITE);  // Clear the screen

        // Draw all boids on the screen
        for (auto& boid : boids) {
            boid.draw();
        }

        EndDrawing();  // End drawing
    }

    CloseWindow();  // Close the window
    return 0;
}
