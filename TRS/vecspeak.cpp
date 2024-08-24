#include <iomanip>
#include <iostream>
#include <vector>

using namespace std;

template <typename T>
void print(T const &value) { cout << value << endl; }
void print(string str) { cout << str << endl; }
void print() { cout << endl; }


int main(){
    print();
    vector<float> v = {1.2};

    v.push_back(20.028);
    v.push_back(20.028);
    v.push_back(20.028);
    v.push_back(20.028);

    v.shrink_to_fit();
    print(v.capacity());
    print(v.size());

    return 0;
}