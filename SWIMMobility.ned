package inet.mobility.single;
import inet.mobility.base.MovingMobilityBase;

simple SWIMMobility extends MovingMobilityBase{
    parameters:
        @signal[updateSignal](type=string);
        
        bool initFromDisplayString = default(true);
        
        int noOfLocations = default(20);
        int Hosts = default(Hosts);
        int popularityDecisionThreshold = default(7); //0-10
        int returnHomePercentage = default(15); //0-100
        int neighbourLocationLimit @unit(m) = default(300m);
        
        double speed @unit(mps) = default(20mps);
        double initialX @unit(m) = default(uniform(constraintAreaMinX, constraintAreaMaxX));
        double initialY @unit(m) = default(uniform(constraintAreaMinY, constraintAreaMaxY));
        double initialZ @unit(m) = default(0m);//default(uniform(constraintAreaMinZ, constraintAreaMaxZ));
        double maxAreaX @unit(m) = default(constraintAreaMaxX);
        double maxAreaY @unit(m) = default(constraintAreaMaxY);
        double maxAreaZ @unit(m) = default(0m);//(constraintAreaMaxZ);
        double radius @unit(m)= default(5m);
        double alpha = default(0.5);
        
        volatile double waitTime @unit(s) = default(0s);
        
        @class (SWIMMobility);
}
