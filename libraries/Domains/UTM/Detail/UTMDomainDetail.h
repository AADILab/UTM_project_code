// Copyright 2016 Carrie Rebhuhn

#ifndef DOMAINS_UTM_UTMDOMAINDETAIL_H_
#define DOMAINS_UTM_UTMDOMAINDETAIL_H_

// STL includes
#include <map>
#include <string>
#include <list>
#include <vector>

// Library includes
#include "Domains/UTM/UTMDomainAbstract.h"
#include "SectorDetail.h"

class UTMDomainDetail : public UTMDomainAbstract {
public:
    UTMDomainDetail(std::string config_file);
    virtual ~UTMDomainDetail() {};

private:
    // Modified objects for child class
    std::vector<SectorDetail*> sectors;
    std::list<UAVDetail*> UAVs;
    std::map<int, std::list<UAVDetail*> > UAVs_done;
    double conflict_thresh;
    // Base function overloads
    virtual matrix1d getRewards();
    virtual matrix1d getPerformance();
    virtual void getPathPlans();  // note: when is this event?
    void getPathPlans(const std::list<UAVDetail*> &new_UAVs);
    virtual void exportLog(std::string fid, double G);
    virtual void detectConflicts();
    virtual void incrementUAVPath();
    virtual void reset();

    bool keep;

    // maps/Graph
    MultiGraph<GridGraph>* lowGraph;

    void addConflict(UAV* u1, UAV* u2) {
        agents->metrics.at(u1->get_cur_sector()).local[u1->get_type()] += 0.5;
        agents->metrics.at(u2->get_cur_sector()).local[u2->get_type()] += 0.5;
    }
    size_t getSector(easymath::XY p);

    // UAV motion tracking
    void logUAVLocations();

    //! UAVLocation is nUAVs*2 x nSteps long, with the first dimension being
    //! twice as long because there are x- and y-values
    matrix2d UAVLocations;
    void exportUAVLocations(int fileID);

    //~B
    virtual void try_to_move(std::vector<UAVDetail*> * eligible_to_move);
    virtual void absorbUAVTraffic();
    virtual void getNewUAVTraffic();

    //~ C+B
    virtual void simulateStep(matrix2d agent_actions) {
        UTMDomainAbstract::simulateStep(agent_actions);
        //exportUAVLocations(neural_net_ID);
    }
};
#endif  // DOMAINS_UTM_UTMDOMAINDETAIL_H_
