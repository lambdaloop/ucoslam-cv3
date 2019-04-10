/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#include "map_types/covisgraph.h"
#include "basictypes/hash.h"
#include <fstream>
#include <iostream>
#include <list>
namespace ucoslam
{
CovisGraph::CovisGraph(){

}
CovisGraph::CovisGraph(      CovisGraph &G){
    std::unique_lock<std::mutex> locker1(G.lock_mutex);
    std::unique_lock<std::mutex> locker2(lock_mutex);

    _nodes=G._nodes;
    _mgraph=G._mgraph;
    _mweights=G._mweights;
}
CovisGraph & CovisGraph::operator=(      CovisGraph &G){
    std::unique_lock<std::mutex> locker1(G.lock_mutex);
    std::unique_lock<std::mutex> locker2(lock_mutex);

    _nodes=G._nodes;
    _mgraph=G._mgraph;
    _mweights=G._mweights;
    return *this;
}
void CovisGraph::clear(){
    _nodes.clear();
    _mgraph.clear();
    _mweights.clear();
}

void CovisGraph::createIncreaseEdge(uint32_t idx1, uint32_t idx2,float increaseValue){
    assert( idx1!=idx2);
    std::unique_lock<std::mutex> locker(lock_mutex);
    _nodes.insert(idx2);
    _nodes.insert(idx1);
    _mgraph[idx1].insert(idx2);
    _mgraph[idx2].insert(idx1);
    auto edge=join(idx1,idx2);
    if (_mweights.count(edge)==0)//needs to be created
        _mweights[edge]=increaseValue;
    else _mweights[edge]+=increaseValue;

}

bool CovisGraph::decreaseRemoveEdge(uint32_t idx1, uint32_t idx2){
    assert( idx1!=idx2);
    auto edge=join(idx1,idx2);

    if (_mweights.count(edge)==0)//needs to be created
        throw std::runtime_error("unexisting edge");

    float &v=_mweights.at(edge);
    if( v>1) //must decrease
    {
        v--;
        return false;
    }
    else {
        _mweights.erase(edge);
        _mgraph.at(idx1).erase(idx2);
        _mgraph.at(idx2).erase(idx1);
        return true;
    }

}
bool  CovisGraph::isEdge(uint32_t idx1, uint32_t idx2  ){

    return _mweights.count(join(idx1,idx2));
}

float CovisGraph::getEdge(uint32_t idx1, uint32_t idx2){
    assert( idx1!=idx2);
    assert(_mweights.count(join(idx1,idx2)));
    return _mweights.at(join(idx1,idx2));
}

std::vector<std::pair<uint32_t,uint32_t> > CovisGraph::getAllEdges(float minWeight){
    std::vector<std::pair<uint32_t,uint32_t> > res;
    res.reserve(_mweights.size());
    for(auto edge:_mweights){
        if (edge.second>minWeight)
            res.push_back( separe(edge.first) );
    }
    return res;
}

std::vector<uint32_t> CovisGraph::getNeighborsV(uint32_t idx,bool includeIdx,float minWeight)  {
    std::unique_lock<std::mutex> locker(lock_mutex);
    auto it=_mgraph.find(idx);
    if (it==_mgraph.end()) return {};//not found
    vector<uint32_t> v;
    v.reserve(it->second.size()+1);
    for(auto p:it->second) {
        bool add=false;
        if (minWeight>0){
         if( getWeight( idx,p )>minWeight) add=true;
        }
        else add=true;
        if(add) v.push_back(p);
    }
    if (includeIdx) v.push_back(idx);
    return v;
}

std::vector<uint32_t> CovisGraph::getNeighborsVLevel2(uint32_t idx,bool includeIdx){
    auto setv=getNeighborsLevel2(idx,includeIdx);
    vector<uint32_t> vv;vv.reserve(setv.size());
    for(auto s:setv)vv.push_back(s);
    return vv;
}

std::set<uint32_t> CovisGraph::getNeighborsLevel2(uint32_t idx,bool includeIdx,float minWeight)  {
    std::set<uint32_t> nl1=getNeighbors(idx,includeIdx,minWeight);
    std::set<uint32_t> nl2;
    for(auto n:nl1){
        std::set<uint32_t> nl=getNeighbors(n,includeIdx,minWeight);
        for(auto n2:nl)
            nl2.insert(n2);
    }
    if (!includeIdx)
        nl2.erase(idx);
    return nl2;
}

std::set<uint32_t> CovisGraph::getNeighbors(uint32_t idx, bool includeIdx, float minWeight)
{
    std::unique_lock<std::mutex> locker(lock_mutex);
    auto it=_mgraph.find(idx);
    if (it==_mgraph.end()){
        if (includeIdx) return{idx};
        else return {};//not found
    }
    if ( !includeIdx && minWeight<=0)
        return it->second;

    std::set<uint32_t> tset;

    if (includeIdx) tset.insert(idx);

    for(auto neigh:it->second)
        if (getWeight(idx,neigh)>=minWeight)
            tset.insert(neigh);
    return tset;
}

std::vector<pair<uint32_t,float> >  CovisGraph::getNeighborsWeights(uint32_t idx, bool sorted){
    std::unique_lock<std::mutex> locker(lock_mutex);
    std::vector<pair<uint32_t,float> > neighs_weights;
    auto it=_mgraph.find(idx);
    if (it==_mgraph.end()) return {};//not found
    //go and save the weights
    neighs_weights.reserve(it->second.size());
    for(auto n:it->second){
        assert(_mweights.count(join(idx,n)));
        neighs_weights.push_back(std::make_pair(n,_mweights[join(idx,n)]));
    }
    //now, sort
    if(sorted)
        std::sort(neighs_weights.begin(),neighs_weights.end(),[](const pair<uint32_t,float> &a,const pair<uint32_t,float> &b){return a.second>b.second;});
    return neighs_weights;
}

void CovisGraph::removeNode(uint32_t idx)
{
    std::unique_lock<std::mutex> locker(lock_mutex);
    _mgraph.erase(idx);
    _nodes.erase(idx);
    //erase all elements in other nodes

    for(auto &g:_mgraph)
        g.second.erase(idx);
    //remove all links having idx
    std::list<uint64_t> toremove;
    for(auto e:_mweights){
        auto l=separe(e.first);
        if( l.first==idx || l.second==idx)
            toremove.push_back(e.first);
    }
    //now, remove the links
    for(auto l:toremove)
        _mweights.erase(l);

}



vector<uint32_t> CovisGraph::getShortestPath(uint32_t org,uint32_t end ){
    //just need to start
    struct TreeNode{
        TreeNode(uint32_t n,uint32_t p):node(n),parent(p){}
        uint32_t node,parent;
    };

    vector<TreeNode> Tree;Tree.reserve(_nodes.size()*4);
    std::vector<bool> visited(_nodes.size(),false);
    Tree.push_back({org,std::numeric_limits<uint32_t>::max()});
    size_t curNode=0;
    bool found=false;
    while( curNode<Tree.size() && !found){
        //take current node
        const auto &tn=Tree[curNode];
        //see ifs neighbors
        for(auto neigh:_mgraph[tn.node]){
            if (!visited[neigh]){
                visited[neigh]=true;
                Tree.push_back(TreeNode(neigh,curNode));
                if ( neigh==end){
                    found=true;break;
                 }
            }
        }
        curNode++;
    }
    if ( !found )//no path
        return {};

    //else ,reconstruct the path in inverse order
    int cnode=Tree.size()-1;
    vector<uint32_t> path;
    while( cnode!=0){
        path.push_back(Tree[cnode].node);
        cnode=Tree[cnode].parent;
    }
    //add the starting node
    path.push_back(org);
    //invert the node order
    std::reverse(path.rbegin(),path.rend());
    return path;
}


//use kruskal algorithm with negative links to obtain the maximum spanning tree
void CovisGraph::getEG(CovisGraph &mst,int minWeight)
{
    mst.clear();
if (_nodes.size()==0)return ;

    //first, add the elements to the  vector
    std::vector<pair<uint64_t,float>> Edges;
    for(auto link:_mweights) Edges.push_back(link);
    //sort in decreasing order
    std::sort(Edges.begin(),Edges.end(),[](const pair<uint64_t,float> &a,const pair<uint64_t,float>& b){return a.second>b.second;});
    std::map<uint32_t,uint32_t> belongs;
    for(auto n:_nodes) belongs.insert({n,n});
    std::vector<uint64_t> mst_edges;//this is in fact the spanning tree
    size_t curEdge=0;
    bool isspanning=false;
    while(curEdge<Edges.size() && !isspanning){
        auto edge=Edges[curEdge++];
        //check that both do not belong to the same tree
        auto i_j=separe(edge.first);

        if ( belongs[i_j.first]!=belongs[i_j.second]){
            //good, add the link, and set all nodes from belongs[i_j.second] to belong now to the first one
            mst_edges.push_back(edge.first);
            auto b1=belongs[i_j.first];
            auto b2=belongs[i_j.second];
            for(auto &b:belongs)
                if ( b.second==b2) b.second=b1;
            //now check if there is only one connected component
            uint32_t cnn=belongs.begin()->second;
            isspanning=true;
            for(auto b:belongs)
                if(b.second!=cnn)  {isspanning=false;break;}
        }
    }

    assert (isspanning);//there are more than one connected components in the graph!!!!

     for(auto m:mst_edges){
        auto i_j=separe(m);
        mst.addEdge(i_j.first,i_j.second,_mweights[m]);
    }

     //now, add the edges above a minimum threshold
     for(auto ew:_mweights){
         if (ew.second>minWeight){
             if(mst._mweights.count(ew.first)==0){//is edge?
                 auto i_j=separe(ew.first);
                 mst.addEdge(i_j.first,i_j.second,ew.second);
             }
         }
     }

}

void CovisGraph::toStream(ostream &str) const
{
    //write nodes

    auto writeSet=[](ostream &str,const std::set<uint32_t> &ss){
        uint32_t s=ss.size();
        str.write((char*)&s,sizeof(s));
        for(const auto &n:ss)  str.write((char*)&n,sizeof(n));
    };


    writeSet(str,_nodes);
    //write map
    uint32_t s=_mgraph.size();    str.write((char*)&s,sizeof(s));
    for(auto m:_mgraph){
        str.write((char*)&m.first,sizeof(m.first));
        writeSet(str,m.second);
    }
    //write weights

    s=_mweights.size();    str.write((char*)&s,sizeof(s));
    for(auto m:_mweights){
        str.write((char*)&m.first,sizeof(m.first));
        str.write((char*)&m.second,sizeof(m.second));
    }
}

void CovisGraph::fromStream(istream &str)
{
    //write nodes

    auto readSet=[](istream &str,  std::set<uint32_t> &ss){
        uint32_t s;
        str.read((char*)&s,sizeof(s));
        for(uint32_t i=0;i<s;i++){
            uint32_t val;
            str.read((char*)&val,sizeof(val));
            ss.insert(val);
        }
    };
    readSet(str,_nodes);
    //write map
    uint32_t s; str.read((char*)&s,sizeof(s));
    for(uint32_t i=0;i<s;i++){
        std::pair< uint32_t,std::set<uint32_t> > m;
        str.read((char*)&m.first,sizeof(m.first));
        readSet(str,m.second);
        _mgraph.insert(m);
    }
    //write weights

    str.read((char*)&s,sizeof(s));
    for(uint32_t i=0;i<s;i++){
        std::pair<uint64_t,float> m;
        str.read((char*)&m.first,sizeof(m.first));
        str.read((char*)&m.second,sizeof(m.second));
        _mweights.insert(m);
    }

}

void CovisGraph::addEdge(uint32_t idx1, uint32_t idx2, float w)
{
    if( idx1==idx2) return ;
     uint32_t  m,M;//set in m the minimum and in M the maximum
    if (idx1>idx2) {m=idx2;M=idx1;}
    else{ m=idx1;M=idx2;}
    _nodes.insert(idx1);
    _nodes.insert(idx2);
    _mgraph[m].insert(M);//insert M as part of m
    _mgraph[M].insert(m);//insert M as part of m
    _mweights[join(idx1,idx2)]=w;//insert the weight in the link set
}
uint64_t CovisGraph::getSignature()const{
    Hash sig;
    sig.add(_nodes.begin(),_nodes.end());
    for(const auto &mg:_mgraph){
        sig+=mg.first;
        sig.add(mg.second.begin(),mg.second.end());
    }
    //write weights
    for(const auto &mg:_mweights){
        sig+=mg.first;
        sig+=mg.second;
    }
    return sig;
}

} // namespace
