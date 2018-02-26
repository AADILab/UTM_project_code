%% Main program for testing MATLAB implementation of multiagent UTM team

clear variables ;

%% UTM team configuration
run = 0 ;
best_team = 0 ;
n_agents = 57 ;
alpha = 10 ; % from config.yaml file, represents longest link length
nn_dir = sprintf('./neural_nets/net_%i_',run) ;

%% Read in agent NN weights
agents(1,n_agents) = SingleHiddenLayerNeuralNet() ;
for i = 1:n_agents
    data = csvread(sprintf('%s%i.csv',nn_dir,i-1)) ;
    r = best_team*2 + 1 ;
    weights = data(r:r+1,:) ;
    agents(i) = SingleHiddenLayerNeuralNet(weights,alpha) ;
end

fprintf('   Agent  0 robots   1 robot  2 robots\n') ;
for i = 1:n_agents
    a = zeros(1,3) ;
    for n_robots = 0:2
        a(n_robots+1) = agents(i).Policy(n_robots) ;
    end
    fprintf('%8.0i%10.3f%10.3f%10.3f\n',i,a(1),a(2),a(3)) ;
end
