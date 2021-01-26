import java.util.ArrayList;
import java.util.Random;

public class ParticleFilter {

  // ===================================================
  // Instance Variables
  
  private final int INITIALNUMPARTICLES = 400;	// Initial number of particles in the filter

  private MapModel mapModel;                      // Details of the mapModel
  private ArrayList<Particle> myParticleSet;      // array of particles
  private ArrayList<Particle> cachedParticleSet;	// 2nd array of particles generated during the updates
  private int numParticles;				// number of particles - we use this version in the code
                                                  // instead of the constant in case we want to vary later
  private Random rand;
  
  public ParticleFilter(MapModel map) {

    // Set up the class
    this.myParticleSet = new ArrayList<Particle>();
    this.cachedParticleSet = new ArrayList<Particle>();
    this.numParticles = INITIALNUMPARTICLES;            // Set this number based on an initial number.
    this.mapModel = map;
//    this.myRobotMonitor = robot;
    this.rand = new Random();
//    this.motionModel = new MotionModel(NOISE_PROFILE);

    double xrange = this.mapModel.getCellWidth() * this.mapModel.getMapWidthInCells();
    double yrange = this.mapModel.getCellHeight()* this.mapModel.getMapHeightInCells();

    // We initialise the array or particles with default values
    for (int i=0; i<this.numParticles;i++) 
      this.myParticleSet.add(this.generateRandomParticle(71.0/2.0,xrange, yrange));
      // NOTE THAT THIS NUMBER SHOULD BE PARAMETERISED AND IS BASED ON THE ROBOT_DIAMETER
    
  }
  
  public Particle generateRandomParticle(double robot_radius, double xrange, double yrange) {
    boolean success=false;
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    while(success!=true) {
      x = rand.nextDouble()*xrange;
      y = rand.nextDouble()*yrange;
      theta = rand.nextDouble()*2.0*Math.PI - Math.PI;

      if(this.mapModel.get_nearest_map_feature_dist(x, y) > robot_radius)
        success=true;
    }
    return new Particle(x, y, theta, 1.0/this.numParticles);
  }

  public ArrayList<Particle> getParticleSet() {
    return this.myParticleSet;
  }

  public int getSize() {
    return this.numParticles;
  }
        
  // ==============================================================================
  // We'll generate the mean pose of the particles
  public Particle getMeanParticle() {
    double xt = 0;      // accumulator for x position
    double yt = 0;      // accumulator for y position
    double xuv = 0;      // accumulator for x unit vector
    double yuv = 0;      // accumulator for y unit vector

    double mySize = (double) this.numParticles;  // possibly not needed?

    for (Particle p:this.myParticleSet) {
      // Average over the values of x and y 
      xt += p.getX();   
      yt += p.getY();   
                        
      // For the heading, as the range is -pi to +pi, we can 
      // generate unit vectors instead and average over these
      xuv += Math.cos(p.getTheta());
      yuv += Math.sin(p.getTheta());
    }           

    // Note that we simply assign an average weight to this particle
    Particle mp = new Particle(xt/mySize,
                  yt/mySize,                                            
                  Math.atan2(yuv/mySize, xuv/mySize),                   
                  1.0/mySize);                                          
    return mp;  
  }     

  // ==============================================================================
  // Resampling involves replacing all of our old particles with newly sampled ones
  // we use the notion of a roulette wheel to model the algorithm used in the COMP329
  // notes - i.e. the stochastic_universal_sampling algorithm
  public void resampleParticles() {
                    
    ArrayList<Particle> resampledParticleset = new ArrayList<Particle>();
    int i,j;  // Indices        
    Particle p,q;
    // Make use of the class's random number generator
    double random = Math.random() * (1.0/numParticles);        
    
    double[] cumulative_weights = new double[numParticles];
    cumulative_weights[0] = this.myParticleSet.get(0).getWeight();
        
    for(int k=1; k < numParticles; k++){ 
      cumulative_weights[k] = cumulative_weights[k-1] + this.myParticleSet.get(k).getWeight();
    }    
    
    i = 0;
    for(j=1; j <= numParticles; j++){ 
    
      while((random + ((j - 1.0) / numParticles )) >= cumulative_weights[i]){
        i++;
      }
      q = this.myParticleSet.get(i);
      q.setWeight(1.0 / numParticles);
      resampledParticleset.add(q);
    }  

    // =================================
    // ...
    // ...
    // Insert Code Here
    // ...
    // ... 
    // Construct a new particle set by randomly sampling from the existing
    // particles, based on the particle weights.  Use the approach for the
    // stochastic_universal_sampling algorithm as duscussed in the notes
    // UPDATE - A modified version of the algorithm has been included with
    // the Assignment Details, as there was an error in the slides that were
    // recorded
    // Once the new set of particles have been created, replace the old ones
    // with the new ones.  Example code is given below
    // =================================
    
    // Clean up - possibly not needed, but should ensure GC
     this.myParticleSet.clear();
     this.myParticleSet = resampledParticleset;
  }
 
  // ==============================================================================
  // Cached Particle Set support
  public void resetCachedParticleset() {
    this.cachedParticleSet.clear();
  }
  
  public void addToCachedParticleSet(Particle pp) {
    this.cachedParticleSet.add(pp);
  }

  public void update_particleset_with_cached(double eta) {
    // Replace the particles in the filter with those in the cache
    // also divide the weights by the normaliser, eta
    this.myParticleSet.clear();
    for (Particle p:this.cachedParticleSet) {
      this.myParticleSet.add(new Particle(p.getX(), p.getY(), p.getTheta(), (p.getWeight()/eta)));
    }
    this.cachedParticleSet.clear();
  }

}