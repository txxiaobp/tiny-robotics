#include "csv_operate.h"
#include <fstream>


CSVOperate::CSVOperate(std::string fileName)
    : fileName(fileName)
{

}

bool CSVOperate::exportToFile(std::vector<std::pair<double,double>> &vec)
{
    std::ofstream of(fileName);
    if (!of.is_open())
    {
        return false;
    }

    for (auto pair : vec)
    {
        of << pair.first << "," << pair.second << std::endl;
    }
    of.close();
    return true;
}
