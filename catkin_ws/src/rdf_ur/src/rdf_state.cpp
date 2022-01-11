#include "rdf_ur/rdf_state.h"

namespace rdf {

    state::state();
        
        public:
            state();

        private:
            bool is_sim;

            virtual bool is_robot_simulated();
            virtual void set_robot_state_pubs();

    };
    
};
