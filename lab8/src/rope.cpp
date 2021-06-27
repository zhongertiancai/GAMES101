#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
    	if (num_nodes == 0 || num_nodes == 1)
    		return;
    	Vector2D p = start;
    	Mass* point = new Mass(p, node_mass, false);
    	masses.push_back(point);
    	for (int i = 0; i < num_nodes - 1; i++) {
    		p = p - start / num_nodes + end / num_nodes;
    		Mass* next = new Mass(p, node_mass, false);
    		Spring* spr = nullptr;
    		spr = new Spring(point, next, k);
    		point = next;
    		masses.push_back(next);
    		springs.push_back(spr);
    	}
//        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }

    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D f, pd;
            pd = s->m2->position - s->m1->position;
            f = s->k * (pd) / (pd.norm()) * (pd.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces -= f;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
            	m->forces += gravity * m->mass;
                // TODO (Part 2): Add global damping
                float k_d_global = 0.01;
                m->forces += - k_d_global * m->velocity;
                Vector2D a = m->forces / m->mass;
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            Vector2D ab = s->m2->position - s->m1->position;
            Vector2D f = s->k *  (ab / ab.norm()) * (ab.norm() - s->rest_length);
            s->m1->forces += f;
            s->m2->forces -= f;        
        }
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
            	m->forces += gravity * m->mass;
                Vector2D a = m->forces / m->mass;
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
                float dampfactor = 0.00005;
                m->position = m->position +  (1 - dampfactor) * (m->position - m->last_position) + a * delta_t *delta_t;
                m->last_position = temp_position;
            }
            m->forces =  Vector2D(0,0);
        }
    }
}
