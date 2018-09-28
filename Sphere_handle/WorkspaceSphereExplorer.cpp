//
// Created by vasko on 23.07.18.
//
//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include <chrono>
#include "WorkspaceSphereExplorer.h"

//#define PRINT_WORKSPACE_PATH

namespace rl
{
    namespace plan
    {
        WorkspaceSphereExplorer::WorkspaceSphereExplorer() :
                goal(),
//              greedy(GREEDY_SPACE), //In the publication there is GREEDY_DISTANCE described
                greedy(GREEDY_DISTANCE),
                model(nullptr),
                radius(0.0),
                range(::std::numeric_limits< double>::max()),
                samples(10),
                start(),
                begin(nullptr),
                end(nullptr),
                graph(),
                queue(),
                meshOpenMesh(), //Added for visualisation
                Sphere(1),      //Initialization for the Sphere visualisation
                rand(
                        ::std::mt19937(::std::random_device()()),
                        ::boost::uniform_on_sphere< double>(3)
                )

        {
        }

        WorkspaceSphereExplorer::~WorkspaceSphereExplorer()
        {
        }

        WorkspaceSphereExplorer::Edge
        WorkspaceSphereExplorer::addEdge(const Vertex& u, const Vertex& v)
        {
            Edge edge = ::boost::add_edge(u, v, this->graph).first;
            //add_edge is returning the edge descriptor and bool value

            return edge;
        }

        WorkspaceSphereExplorer::Vertex
        WorkspaceSphereExplorer::addVertex(const WorkspaceSphere& sphere)
        {
            Vertex vertex = ::boost::add_vertex(this->graph);
            this->graph[vertex].sphere = sphere;
            this->Sphere.addSphere(sphere.radius,*sphere.center);
            return vertex;
        }

        bool
        WorkspaceSphereExplorer::explore()
        {
            WorkspaceSphere start;
            start.center = ::std::make_shared< Vector3>(*this->start);
            //this->start I should have this initialized so i know what my start is
//            start.radius = this->model->distance(*start.center); //From original
            start.radius =FCL_DistanceToCollision::Callculate(start.center,model); //FCL distance calculation
            start.radiusSum = start.radius;
            start.parent = nullptr;

            start.priority = (*this->goal - *start.center).norm() - start.radius;

            this->queue.insert(start);

            while (!this->queue.empty())
            {
                WorkspaceSphere top = *this->queue.begin();

                this->queue.erase(this->queue.begin());

                if (top.radius >= this->radius)
                {
                    Vertex vertex = this->addVertex(top);

                    if (nullptr != top.parent)
                    {
                        this->addEdge(top.parent, vertex);
                    }
                    else
                    {
                        this->begin = vertex;
                    }

                    if ((*this->goal - *top.center).norm() < top.radius)
                    {
                        WorkspaceSphere goal;
                        goal.center = ::std::make_shared< Vector3>(*this->goal);
//                        goal.radius = this->model->distance(*goal.center); //From original file
                        goal.radius =FCL_DistanceToCollision::Callculate(goal.center,model); //FCL distance calculation
                        goal.parent = vertex;
                        goal.priority = (*this->goal - *goal.center).norm() - goal.radius;

                        this->end = this->addVertex(goal);

                        if (nullptr != top.parent)
                        {
                            this->addEdge(top.parent, this->end);
                        }
                        else
                        {
                            this->addEdge(this->begin, this->end);
                        }

                        this->meshOpenMesh=this->Sphere.get_mesh();
                        return true;
                    }

                    ::std::multiset<WorkspaceSphere>::iterator i = this->queue.begin();
                    ::std::multiset<WorkspaceSphere>::iterator j;

                    while (i != this->queue.end())
                    {
                        if ((*i->center - *top.center).norm() < top.radius)
                        {
                            j = i;
                            ++i;
                            this->queue.erase(j);
                        }
                        else
                        {
                            ++i;
                        }
                    }
                    //Std::ceil computes the closest INTEGER value not less the given
                    //In this case this->samples is some constant depending on the radius of the sphere
                    //so we will have bigger amount of samples for the bigger spheres
                    for (::std::size_t i = 0; i < ::std::ceil(this->samples * top.radius); ++i)
//for (::std::size_t i = 0; i < this->samples; ++i) // TODO
                    {
                        WorkspaceSphere sphere;

                        sphere.parent = vertex;

                        ::boost::uniform_on_sphere< double>::result_type sample = this->rand();

                        sphere.center = ::std::make_shared< Vector3>(
                                top.radius * Vector3(sample[0], sample[1], sample[2]) + *top.center // TODO
                        );

                        if ((*this->start - *sphere.center).norm() <= this->range) //Checking if the robot can reach
                        {
                            //It continues if sphere.center is NOT inside of some of previous sphere
                            if (!this->isCovered(top.parent, *sphere.center))
                            {
//                                sphere.radius = this->model->distance(*sphere.center); //From original
                                sphere.radius =FCL_DistanceToCollision::Callculate(sphere.center,model); //FCL distance calculation
                                sphere.radiusSum = sphere.radius + top.radiusSum;

                                if (sphere.radius >= this->radius)
                                {
                                    switch (this->greedy)
                                    {
                                        case GREEDY_DISTANCE:
                                            sphere.priority = (*this->goal - *sphere.center).norm() - sphere.radius;
                                            break;
                                        case GREEDY_SOURCE_DISTANCE:
                                            sphere.priority = (*this->goal - *sphere.center).norm() - sphere.radius + top.radiusSum;
                                            break;
                                        case GREEDY_SPACE:
                                            sphere.priority = 1.0f / sphere.radius;
                                            break;
                                        default:
                                            break;
                                    }

                                    this->queue.insert(sphere);
                                }
                            }
                        }
                    }
                }
            }

            return false;
        }

        WorkspaceSphereList
        WorkspaceSphereExplorer::getPath() const
        {
            WorkspaceSphereList path;

            Vertex i = this->end;

            while (i != this->begin)
            {
                path.push_front(this->graph[i].sphere);

                i = ::boost::source(*::boost::in_edges(i, this->graph).first, this->graph);
            }

            path.push_front(this->graph[i].sphere);
            return path;
        }

        bool
        WorkspaceSphereExplorer::isCovered(const Vector3& point) const
        {
            for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
            {
                if ((point - *this->graph[*i.first].sphere.center).norm() < this->graph[*i.first].sphere.radius)
                {
                    return true;
                }
            }

            return false;
        }

        bool
        WorkspaceSphereExplorer::isCovered(const Vertex& parent, const Vector3& point) const
        {
            //Iterate over all the spheres starting from first to last(last is not concidered)
            //boost vertices is returning a pair,which contain the first and the last sphere
            for (VertexIteratorPair i = ::boost::vertices(this->graph); i.first != i.second; ++i.first)
            {
                //Checking for the chose sphere not to be top sphere form the main program
                if (parent != this->graph[*i.first].sphere.parent)
                {
                    //if condition which we didnt understand in the pseudo code
                    if ((point - *this->graph[*i.first].sphere.center).norm() < this->graph[*i.first].sphere.radius)
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        void
        WorkspaceSphereExplorer::reset()
        {
            this->graph.clear();
            this->queue.clear();
            this->begin = nullptr;
            this->end = nullptr;
        }

        void
        WorkspaceSphereExplorer::seed(const ::std::mt19937::result_type& value)
        {
            this->rand.engine().seed(value);
        }
    }
}
