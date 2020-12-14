
#ifndef DEMA_FILTER
#define DEMA_FILTER
/*
   Author Piotr Nikiel

   Based on https://www.norwegiancreations.com/2016/08/double-exponential-moving-average-filter-speeding-up-the-ema/
 */

class EMAFilter {

protected:
float _alpha;
float _output;

public:
EMAFilter(float alpha, float initialOutput = 0) : _output(initialOutput){
        setAlpha(alpha);
}

EMAFilter(const EMAFilter &filter){
        _alpha = filter.getAlpha();
        _output = filter.getOutput();
}

float getOutput (void) const {
        return _output;
}

float getAlpha (void) const {
        return _alpha;
}

float filter(float data){
        _output = (1.0f - _alpha) * _output + _alpha * data;
        return _output;
}

void setAlpha(float alpha){
        alpha = (alpha<0) ? -alpha : alpha;
        _alpha = (alpha>1.0) ? 1.0 : alpha;
}

void setOutput(float output){
        _output = output;
}


};

class DEMAFilter {
protected:
float _output;
float _alpha;
EMAFilter _filter;
EMAFilter _recursiveFilter;

public:
DEMAFilter(float alpha, float initialOutput = 0) : _filter(alpha,initialOutput),_recursiveFilter(alpha,initialOutput){
        setAlpha(alpha);
        setOutput(initialOutput);
}

DEMAFilter(const EMAFilter &filter) : DEMAFilter(filter.getAlpha(),filter.getOutput()){
}

float filter(float data){
        _filter.filter(data); // filter data base class
        _recursiveFilter.filter(_filter.getOutput()); // filter filtered data

        _output = 2*_filter.getOutput()-_recursiveFilter.getOutput();     // combine above
        return _output;
}

float getOutput (void) const {
        return _output;
}

float getAlpha (void) const {
        return _alpha;
}

void setAlpha(float alpha){
        alpha = (alpha<0) ? -alpha : alpha;
        _alpha = (alpha>1.0) ? 1.0 : alpha;

        _filter.setAlpha(_alpha);
        _recursiveFilter.setAlpha(_alpha);
}

void setOutput(float output){
        _output = output;
        _filter.setOutput(_output);
        _recursiveFilter.setOutput(_output);
}

};




#endif /* end of include guard: DEMA_FILTER */
