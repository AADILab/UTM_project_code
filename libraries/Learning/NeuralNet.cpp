// Copyright 2016 Carrie Rebhuhns
#include "NeuralNet.h"

using easymath::rand;
using easymath::sum;
using easymath::operator*;
using easymath::operator-;
using easymath::operator+;
using easymath::dot;
using easymath::flatten;
using easymath::sigmoid;
using easymath::sigmoidDerivative;
using easymath::randSetFanIn;
using easymath::T;
using std::vector;
using std::string;


Action NeuralNet::operator()(State X) const {
  X.push_back(1.0);

  auto s = X*layers_[0].w_bar_;
  auto z = sigmoid(s);

  for (size_t i = 1; i < N; i++) {
    auto zbias = z;
    zbias.push_back(1.0);
    s = zbias*layers_[i].w_bar_;
    z = sigmoid(s);
  }
  if (sigmoid_output_)
    return z;
  else
    return s;
}

void NeuralNet::updateWt(matrix1d X, const BPState& S) {
  X.push_back(1.0);
  layers_[0].w_bar_ = layers_[0].w_bar_ - k_eta_*T(S.d[0] * X);
  for (size_t i = 0, j = 1; j < N; i++, j++) {
    matrix1d zbias = S.z[i];
    zbias.push_back(1.0);
    matrix2d w_grad = - k_eta_ * T(S.d[j] * zbias);
    layers_[j].w_bar_ = layers_[j].w_bar_ + w_grad;
  }
}

void NeuralNet::backProp(matrix1d Y, BPState *S) {
  S->d = matrix2d(N);
  S->d[N - 1] = S->z[N - 1] - Y;

  int i = 0;
  auto w_nobias = layers_[i].w_;
  matrix2d m = w_nobias[i] * S->d[i + 1];

  S->d[i] = dot(S->f[i], flatten(m));
}

matrix1d NeuralNet::fwrdProp(matrix1d X, BPState* S) {
  X.push_back(1.0);
  if (S == NULL) {
    S = new BPState();
  }

  S->s[0] = X*layers_[0].w_bar_;
  S->z[0] = sigmoid(S->s[0]);
  S->f[0] = sigmoidDerivative(S->s[0]);

  for (size_t i = 1; i < N; i++) {
    auto zbias = S->z[i - 1];
    zbias.push_back(1.0);
    S->s[i] = zbias*layers_[i].w_bar_;
    S->z[i] = sigmoid(S->s[i]);
    S->f[i] = sigmoidDerivative(S->s[i]);
  }
  return S->z[N - 1];
}


double NeuralNet::backProp(const matrix1d &X, const matrix1d &Y) {
  // 'X' is the input vector, 'Y' is the 'target vector'
  // returns the SSE for the output vector

  fwrdProp(X, nn_state_);
  backProp(Y, nn_state_);
  updateWt(X, *nn_state_);

  matrix1d Yactual = operator()(X);
  matrix1d DY = Yactual - Y;
  double sse = 0.0;
  for (double dy : DY) {
    sse += dy*dy;
  }
  return sse;
}

double NeuralNet::backPropBatch(matrix2d X, matrix2d Y, int iterations) {
  std::vector<size_t> indices(X.size());  // size of X and Y should be same
  for (size_t i = 0; i < X.size(); i++)
    indices[i] = i;
  
  double MSE = 0.0;
  for (size_t it = 0; it < iterations; it++) {
    MSE = 0.0;
    std::random_shuffle(indices.begin(), indices.end());
    for (auto i:indices)
      MSE += backProp(X[i], Y[i]);
    MSE /= double(X.size());
  }
  return MSE;  // only returns the LAST mse
}

NeuralNet::Layer::Layer(size_t above, size_t below) :
  k_num_nodes_above_(above), k_num_nodes_below_(below) {
  // Populate Wbar with small random weights, including bias
  w_bar_ = easymath::zeros(above + 1, below);

  for (size_t i = 0; i < w_bar_.size(); i++) {
    for (size_t j = 0; j < w_bar_[i].size(); j++) {
      w_bar_[i][j] = easymath::rand(-1.0, 1.0);
    }
  }

  w_ = w_bar_;
  w_.pop_back();
}

void NeuralNet::mutate(int new_id) {
  NNID = new_id;
  for (Layer &l : layers_) {
    for (matrix1d &wt_outer : l.w_bar_) {
      for (double &w : wt_outer) {
        w += mutateAmount();
      }
    }
  }
	updateMinMax();
}

double NeuralNet::mutateAmount() {
  // Adds random amount mutRate_% of the time,
  if (rand(0, 1) > k_mut_rate_) {
    return 0.0;
  } else {
    double result = (pow(-1, rand() % 2))*normal_(generator);
    return result;
  }
}

NeuralNet::NeuralNet(YAML::Node configs, int id_num): min_w_(INFINITY), max_w_(-INFINITY), NNID(id_num), firstEvaluation(true) {
  k_gamma_ = configs["nn"]["gamma"].as<double>();
  k_eta_ = configs["nn"]["eta"].as<double>();
  k_mut_rate_ = configs["nn"]["mut_rate"].as<double>();
  k_mut_std_ = configs["nn"]["mut_std"].as<double>();
  N = configs["nn"]["layers"].as<size_t>()+1;
  sigmoid_output_ = configs["nn"]["sigmoid_output"].as<bool>();
  size_t num_hidden = configs["nn"]["hidden"].as<size_t>();
  size_t num_inputs = configs["nn"]["input"].as<size_t>(); // note, this is a derived value
  size_t num_outputs = configs["nn"]["output"].as<size_t>(); // note, this is a derived value

  evaluation_ = 0;
  
  nn_state_ = new BPState();

  // 1 hidden layer neural net (corresponds to N of 2)
  layers_.push_back(Layer(num_inputs, num_hidden));
  layers_.push_back(Layer(num_hidden, num_outputs));
  // generator.seed(0);
	
	updateMinMax();

  normal_ = std::normal_distribution<double>(0.0, k_mut_std_);
}

void NeuralNet::load(string filein) {
  matrix2d wts = cio::read2<double>(filein);
  load(wts[0], wts[1]);
}

void NeuralNet::save(string fileout) const {
  matrix2d out(2);
  out[0] = getTopology();
  for (Layer l : layers_) {
    matrix1d flat_wbar = flatten(l.w_bar_);
    out[1].insert(out[1].end(), flat_wbar.begin(), flat_wbar.end());
  }
  cio::print2(out, fileout, false);
}

// Now must explicitly call load
//NeuralNet::NeuralNet(std::string fname) : NeuralNet() {
//    load(fname);
//}

void NeuralNet::load(matrix1d node_info, matrix1d wt_info) {
  size_t num_inputs = static_cast<int>(node_info[0]);
  size_t num_hidden = static_cast<int>(node_info[1]);
  size_t num_outputs = static_cast<int>(node_info[2]);

  layers_ = std::vector<Layer>();
  layers_.push_back(Layer(num_inputs, num_hidden));
  layers_.push_back(Layer(num_hidden, num_outputs));

  int index = 0;  // index for accessing NN elements
  for (Layer &l : layers_) {  // number of layers
    for (matrix1d &wt_outer : l.w_bar_) {
      for (double &wt_inner : wt_outer) {
        wt_inner = wt_info[index++];
      }
    }
    l.w_ = l.w_bar_;
    l.w_.pop_back();
  }
}


matrix1d NeuralNet::getTopology() const {
  matrix1d topology(1);
  topology[0] = layers_.front().k_num_nodes_above_;
  for (Layer l : layers_)
    topology.push_back(static_cast<double>(l.k_num_nodes_below_));
  return topology;
}

void NeuralNet::updateMinMax() {
	min_w_ = INFINITY;
	max_w_ = -INFINITY;
	for (Layer &l : layers_) {
		for (matrix1d &wt_outer : l.w_bar_) {
			for (double &w : wt_outer) {
				if (w < min_w_)
					min_w_ = w;
				else if (w > max_w_)
					max_w_ = w;
			}
		}
	}
}

std::pair<double, double> NeuralNet::getWtRange() {
	return std::pair<double, double>(min_w_, max_w_);
}
