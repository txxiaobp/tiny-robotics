#ifndef CSVOPERATE_H
#define CSVOPERATE_H

#include <vector>
#include <string>

class CSVOperate
{
public:
    CSVOperate(std::string fileName = "data.csv");
    bool exportToFile(std::vector<std::pair<double,double>> &vec);

private:
    std::string fileName;
};

#endif // CSVOPERATE_H
