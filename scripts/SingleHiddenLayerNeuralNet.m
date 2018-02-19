classdef SingleHiddenLayerNeuralNet
    properties
        alpha ;
        input2Hidden ;
        hidden2Output ;
    end
    methods
        function obj = SingleHiddenLayerNeuralNet(weights,a)    % constructor
            if nargin > 0
                obj.alpha = a ;                                     % constant factor
                n_in = weights(1,1) ;                               % inputs
                n_hidden = weights(1,2) ;                           % hidden
                n_out = weights(1,3) ;                              % outputs

                obj.input2Hidden = zeros(n_in+1,n_hidden) ;
                obj.hidden2Output = zeros(n_hidden+1,n_out) ;

                obj.input2Hidden = reshape(weights(2,1:numel(obj.input2Hidden)),fliplr(size(obj.input2Hidden)))' ;
                obj.hidden2Output = reshape(weights(2,numel(obj.input2Hidden)+1:end-1),size(obj.hidden2Output)) ;
            end
        end
        function action = Policy(obj,state)
            x = [state, 1] ; % append bias
            s = x*[obj.input2Hidden] ;
            hidden = sigmoid(s) ;
            zbias = [hidden, 1] ; % append bias
            s = zbias*[obj.hidden2Output] ;
            action = sigmoid(s)*obj.alpha ;
        end
    end
end
function z = sigmoid(s)
    z = 1./(1 + exp(-s)) ;
end