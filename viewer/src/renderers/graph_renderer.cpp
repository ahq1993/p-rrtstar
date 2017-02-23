#include "graph_renderer.h"

#include <lcmtypes/lcmtypes.h>

#include <vector>
#include <cmath>

#define RENDERER_NAME "Graph Visualizer"

using namespace std;


class RendererGraph {
public:
    BotRenderer renderer;
    BotGtkParamWidget *pw;
    BotViewer *viewer;
    lcm_t * lcm;
    
    lcmtypes_graph_t *graph_last;
    lcmtypes_environment_t *environment_last;
    
    double obstacle_opacity;
};


        
static void graph_message_handler (const lcm_recv_buf_t *rbuf, const char *channel, const lcmtypes_graph_t *msg, void *user) {
    
    RendererGraph *self = (RendererGraph *) user;
    
    if (self->graph_last) 
        lcmtypes_graph_t_destroy (self->graph_last);
    
    self->graph_last = lcmtypes_graph_t_copy (msg);
    
    bot_viewer_request_redraw (self->viewer);
}


static void environment_message_handler (const lcm_recv_buf_t *rbuf, const char *channel, const lcmtypes_environment_t *msg, void *user) {

    RendererGraph *self = (RendererGraph *) user;
    
    if (self->environment_last) 
        lcmtypes_environment_t_destroy (self->environment_last);
    
    self->environment_last = lcmtypes_environment_t_copy (msg);
    
    bot_viewer_request_redraw (self->viewer);
}


static void renderer_graph_draw(BotViewer *viewer, BotRenderer *renderer)
{   
    
    RendererGraph *self = (RendererGraph*) renderer;

    
    glEnable(GL_DEPTH_TEST);
    
    glEnable (GL_BLEND);
    glEnable (GL_RESCALE_NORMAL);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel (GL_SMOOTH);
    glEnable (GL_LIGHTING);

    
    // Draw the graph
    if (self->graph_last) {
        
        // Draw the vertices
        glPointSize (3.0);
        float color_vertex[] = {0.1, 0.1, 0.8, 1.0};
        glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_vertex);        
        glBegin (GL_POINTS);

        for (int i = 0; i < self->graph_last->num_vertices; i++) {
            
            glVertex3f (self->graph_last->vertices[i].state.x, 
                        self->graph_last->vertices[i].state.y,
                        self->graph_last->vertices[i].state.z);
        }

        glEnd();
        
        
        // Draw the edges
        for (int i = 0; i < self->graph_last->num_edges; i++) {
            
            glLineWidth (2.0); 
            float color_edge[] = {0.8, 0.3, 0.3, 1.0};
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_edge);
            glBegin (GL_LINE_STRIP);
            
            glVertex3f (self->graph_last->edges[i].vertex_src.state.x,
                        self->graph_last->edges[i].vertex_src.state.y,
                        self->graph_last->edges[i].vertex_src.state.z);
            glVertex3f (self->graph_last->edges[i].vertex_dst.state.x, 
                        self->graph_last->edges[i].vertex_dst.state.y,
                        self->graph_last->edges[i].vertex_dst.state.z);
            glEnd();
        }
    }
    
    
    // Draw the environment
    if (self->environment_last) {
        
        // Draw the goal region 

        
        // Draw the obstacles        
        for (int i = 0; i < self->environment_last->num_obstacles; i++) {

            float color_obstacles[] = { 0.3, 0, 0, ((double)(self->obstacle_opacity))/100.0};
            glMaterialfv (GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, color_obstacles);
            
            
            glPushMatrix ();
            
            glTranslated (self->environment_last->obstacles[i].center[0], self->environment_last->obstacles[i].center[1], self->environment_last->obstacles[i].center[2]);
            glRotatef (0.0, 0.0, 0.0, 1.0);
            
            glScalef (self->environment_last->obstacles[i].size[0], self->environment_last->obstacles[i].size[1], self->environment_last->obstacles[i].size[2]);
            
            bot_gl_draw_cube ();
            
            glPopMatrix ();
        }        
    }
    
    
    return;
}


static void renderer_graph_free (BotRenderer *renderer)
{
    RendererGraph *self = (RendererGraph*) renderer;
    
    if (self->graph_last)
        lcmtypes_graph_t_destroy (self->graph_last);
    free(self);
}


void add_graph_renderer_to_viewer (BotViewer* viewer, int render_priority, lcm_t* lcm)
{
    
    RendererGraph *self = new RendererGraph;
    BotRenderer *renderer = &self->renderer;
    self->lcm = lcm;
    self->viewer = viewer;
    
    renderer->draw = renderer_graph_draw;
    renderer->destroy = renderer_graph_free;
    renderer->widget = gtk_vbox_new(FALSE, 0);
    renderer->name = (char *) RENDERER_NAME;
    renderer->user = self;
    renderer->enabled = 1;
    
    
    self->graph_last = NULL;
    self->environment_last = NULL;
    
    self->obstacle_opacity = 50;
    
    // subscribe to messages
    lcmtypes_graph_t_subscribe (lcm, "GRAPH", graph_message_handler, self);
    lcmtypes_environment_t_subscribe (lcm, "ENVIRONMENT", environment_message_handler, self);
    
    bot_viewer_add_renderer(viewer, &self->renderer, render_priority);
}