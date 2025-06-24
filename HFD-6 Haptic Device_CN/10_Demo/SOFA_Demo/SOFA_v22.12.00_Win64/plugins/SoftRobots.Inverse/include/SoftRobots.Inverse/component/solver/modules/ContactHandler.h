#ifndef SOFTROBOTS_INVERSE_CONTACTHANDLER_H
#define SOFTROBOTS_INVERSE_CONTACTHANDLER_H

#include <sofa/type/vector.h>
#include <SoftRobots.Inverse/component/config.h>

namespace sofa {

namespace component {

namespace constraintset {

class SOFA_SOFTROBOTS_INVERSE_API ContactHandler
{
public:
    ContactHandler(){}
    virtual ~ContactHandler(){}

public:

    /// Return if the contact has reached is boundary, different implementation depending on contact state.
    /// Input: contact force, contact violation, radius=friction_coef*contact_normal_force
    /// Output: true if the contact has reached is boundary, else false
    virtual bool hasReachedBoundary(const type::vector<double>& lambda,
                                    const type::vector<double>& delta ,
                                    const double& mu ,
                                    int& candidateId)
    {
        SOFA_UNUSED(mu);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);
        SOFA_UNUSED(candidateId);

        return false;
    }

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta )
    {
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return false;
    }

    /// Find new contact state after pivot and return a pointer to the corresponding ContactHandler
    /// Input: Pointers on different ContactHandler {active, inactive} in case of frictionless or {inactive, stick, sliding} in case of friction
    ///        contact force, contact violation, radius=friction_coef*contact_normal_force
    /// Output: Pointer on new ContactHandler corresponding to new state after pivoting
    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const type::vector<double>& lambda,
                                                 const type::vector<double>& delta ,
                                                 const double& mu )
    {
        SOFA_UNUSED(mu);
        SOFA_UNUSED(ptrList);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return nullptr;
    }

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta )
    {
        SOFA_UNUSED(ptrList);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return nullptr;
    }

    void setAllowSliding(bool allowSliding) {m_allowSliding=allowSliding;}
    virtual std::string getStateString()=0;

protected:
    double m_epsilon{1e-14};
    double m_allowSliding{false};

};


class ActiveContactHandler : public ContactHandler
{
public:
    ActiveContactHandler(){}
    virtual ~ActiveContactHandler(){}

public:

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta ) override;

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta ) override;

    virtual std::string getStateString() {return "Active";}
};


class InactiveContactHandler : public ContactHandler
{
public:
    InactiveContactHandler(){}
    virtual ~InactiveContactHandler(){}

public:
    virtual bool hasReachedBoundary(const type::vector<double>& lambda,
                                    const type::vector<double>& delta ,
                                    const double& mu ,
                                    int& candidateId) override;

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta) override;

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const type::vector<double>& lambda,
                                                 const type::vector<double>& delta ,
                                                 const double& mu) override;

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta ) override;

    virtual std::string getStateString() {return "Inactive";}
};


class StickContactHandler : public ContactHandler
{
public:
    StickContactHandler(){}
    virtual ~StickContactHandler(){}

public:
    virtual bool hasReachedBoundary(const type::vector<double>& lambda,
                                    const type::vector<double>& delta ,
                                    const double& mu,
                                    int& candidateId) override;

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const type::vector<double>& lambda,
                                                 const type::vector<double>& delta ,
                                                 const double& mu) override;

    virtual std::string getStateString() {return "Stick";}
};


class SlidingContactHandler : public ContactHandler
{
public:
    SlidingContactHandler(){}
    virtual ~SlidingContactHandler(){}

public:
    virtual bool hasReachedBoundary(const type::vector<double>& lambda,
                                    const type::vector<double>& delta ,
                                    const double& mu,
                                    int& candidateId) override;

    virtual ContactHandler* getNewContactHandler(const type::vector<ContactHandler*>& ptrList,
                                                 const type::vector<double>& lambda,
                                                 const type::vector<double>& delta ,
                                                 const double& mu) override;

    virtual std::string getStateString() {return "Sliding";}
};

}

}

}

#endif

