#include "matrix.h"

#include <cassert>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <cstdio>


Matrix::Matrix(int row, int col)
    : elem(std::vector<std::vector<double>>(row, std::vector<double>(col, 0)))
{
    assert(row > 0 && col > 0);
}

Matrix::Matrix(std::vector<double> &vector)
    : elem(std::vector<std::vector<double>>(vector.size(), std::vector<double>(1, 0)))
{
    assert(!vector.empty());

    for (decltype (vector.size()) row = 0; row < vector.size(); row++)
    {
        elem[row][0] = vector[row];
    }
}

Matrix::Matrix(std::vector<double> &vector, int row, int col)
    : elem(std::vector<std::vector<double>>(row, std::vector<double>(col, 0)))
{
    assert(row > 0 && col > 0);
    assert(row * col <= int(vector.size()));

    int vecIndex = 0;

    for (int r = 0; r < row; r++)
    {
        for (int c = 0; c < col; c++)
        {
            if (vecIndex >= int(vector.size()))
            {
                elem[r][c] = 0.0;
            }
            else
            {
                elem[r][c] = vector[vecIndex++];
            }
        }
    }
}


Matrix::Matrix(std::vector<double> &vector, int size)
    : elem(std::vector<std::vector<double>>(size, std::vector<double>(size, 0)))
{
    assert(size > 0 && int(vector.size()) <= size);

    for (int row = 0; row < size; row++)
    {
        elem[row][row] = vector[row];
    }
}


/*
 * 属性：Private
 * row：行号
 * col：列号
 * 作用：判断矩阵中的某个元素是否近似为0
*/
bool Matrix::isNearlyZero(int row, int col)
{
    assert ((row >= 0 && row < getRow()) && (col >= 0 || col < getCol()));

    return fabs(elem[row][col]) < EPS;
}

/*
 * 属性：Public
 * startRow：行号
 * 作用：获取某列主元所在行号
*/
int Matrix::pickPivotRow(int startRow)
{
    double tempValue = elem[startRow][startRow];
    int tempRow = startRow;
    for (int i = startRow + 1; i < getRow(); i++)
    {
        if (fabs(elem[i][startRow]) - fabs(tempValue) > EPS)
        {
            tempValue = elem[i][startRow];
            tempRow = i;
        }
    }
    if (fabs(tempValue) < EPS)
    {
        return MATRIX_SINGULAR;
    }
    return tempRow;
}

int Matrix::getRow() const
{
    return elem.size();
}

int Matrix::getCol() const
{
    return elem[0].size();
}

void Matrix::setValue(int row, int col, double value)
{
    assert ((row >= 0 && row < getRow()) && (col >= 0 || col < getCol()));

    elem[row][col] = value;
}


double Matrix::getValue(int row, int col) const
{
    assert ((row >= 0 && row < getRow()) && (col >= 0 || col < getCol()));

    return elem[row][col];
}

void Matrix::setValueReverse(int row, int col)
{
    assert ((row >= 0 && row < getRow()) && (col >= 0 || col < getCol()));

    elem[row][col] = -1.0 * elem[row][col];
}

void Matrix::setValueReciprocal(int row, int col)
{
    assert ((row >= 0 && row < getRow()) && (col >= 0 || col < getCol()));

    elem[row][col] = 1.0 / elem[row][col];
}

void Matrix::rowMulByConst(int row, double factor)
{
    assert (row >= 0 && row < getRow());

    for (int i = 0; i < getCol(); i++)
    {
        elem[row][i] *= factor;
    }
}

void Matrix::addRowToAnother(int dRow, int sRow, double factor)
{
    assert (dRow >= 0 && dRow < getRow());
    assert (sRow >= 0 && sRow < getRow());

    for (int i = 0; i < getCol(); i++)
    {
        elem[dRow][i] += factor * elem[sRow][i];
    }
}

void Matrix::swapTwoRow(int row1, int row2)
{
    assert (row1 >= 0 && row1 < getRow());
    assert (row2 >= 0 || row2 < getRow());

    std::swap(elem[row1], elem[row2]);
}

void Matrix::reducedEchelon()
{
    for (int i = 0; i < getRow(); i++)
    {
        // 选取列主元所在的行
        int pivotRow = pickPivotRow(i);
        assert(pivotRow != MATRIX_SINGULAR);

        // 选取列主元所在的行与当前行交换
        swapTwoRow(pivotRow, i);

        // 选取列主元
        double pivot = getValue(i, i);

        // 列主元所在行除以列主元
        rowMulByConst(i, 1.0 / pivot);

        for (int j = i + 1; j < getRow(); j++)
        {
            // 列主元以下的行变为0
            double rowValue = getValue(j, i);
            addRowToAnother(j, i, -1.0 * rowValue);
        }
    }

    // 现在是上三角矩阵

    for (int i = getRow() - 1; i > 0; i--)
    {
        for (int j = i - 1; j >= 0; j--)
        {
            double rowValue = getValue(j, i);
            addRowToAnother(j, i, -1.0 * rowValue);
        }
    }
    // 现在是对角矩阵
}

Matrix Matrix::reverseMatrix()
{
    assert(getRow() == getCol());
    std::vector<double> vector(getRow(), 1.0);
    Matrix diagMatrix(vector, getRow());

    return this->operator/(diagMatrix);
}

Matrix Matrix::transpose()
{
    Matrix retMatrix(getCol(), getRow());
    for (int r = 0; r < getRow(); r++)
    {
        for (int c = 0; c < getCol(); c++)
        {
            retMatrix.elem[c][r] = elem[r][c];
        }
    }
    return retMatrix;
}

int Matrix::getDetermine()
{
    Matrix tempMatrix(*this);
    double det = 1.0;

    for (int i = 0; i < getRow(); i++)
    {
        int pivotRow = tempMatrix.pickPivotRow(i);
        if (-1 == pivotRow)
        {
            det = 0.0;
            break;
        }

        if (pivotRow != i)
        {
            tempMatrix.swapTwoRow(pivotRow, i);
            det = -det;
        }

        double pivot = tempMatrix.getValue(i, i);
        det *= pivot;

        double mulFactor = -1.0 / pivot;

        for (int j = i + 1; j < getRow(); j++)
        {
            double rowValue = tempMatrix.getValue(j, i);
            tempMatrix.addRowToAnother(j, i, mulFactor * rowValue);
        }
    }

    return det;
}

int Matrix::getRank()
{
    int rank = 0;
    int r = std::min(getRow(), getCol());
    Matrix tempMatrix = Matrix(*this);

    for (int i = 0; i < r; i++)
    {
        int pivotRow = tempMatrix.pickPivotRow(i);
        if (-1 == pivotRow) //该列无主元
        {
            continue;
        }
        rank++;

        tempMatrix.swapTwoRow(pivotRow, i);

        double pivot = tempMatrix.getValue(i, i);
        tempMatrix.rowMulByConst(i, 1 / pivot);

        for (int j = i + 1; j < getRow(); j++)
        {
            double value = tempMatrix.getValue(j, i);
            double factor = -1.0 * value;
            tempMatrix.addRowToAnother(j, i, factor);
        }
    }

    return rank;
}

Matrix Matrix::operator+(const Matrix& other) const
{
    assert(getRow() == other.getRow());
    assert(getCol() == other.getCol());

    Matrix retMatrix(getRow(), getCol());

    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            double sum = getValue(i, j) + other.getValue(i, j);
            retMatrix.setValue(i, j, sum);
        }
    }

    return retMatrix;
}

void Matrix::operator+=(const Matrix& other)
{
    assert(getRow() == other.getRow());
    assert(getCol() == other.getCol());

    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            elem[i][j] += other.elem[i][j];
        }
    }
}

void Matrix::operator*=(const double scaler)
{
    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            elem[i][j] *= scaler;
        }
    }
}

Matrix Matrix::operator-(const Matrix& other) const
{
    assert(getRow() == other.getRow());
    assert(getCol() == other.getCol());

    Matrix retMatrix(getRow(), getCol());

    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            double sub = getValue(i, j) - other.getValue(i, j);
            retMatrix.setValue(i, j, sub);
        }
    }

    return retMatrix;
}

Matrix Matrix::operator*(const double scaler) const
{
    Matrix retMatrix(getRow(), getCol());

    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            double mul = getValue(i, j) * scaler;
            retMatrix.setValue(i, j, mul);
        }
    }

    return retMatrix;
}

Matrix Matrix::operator*(const Matrix& other) const
{
    assert(getCol() == other.getRow());

    int retRow = getRow();
    int retCol = other.getCol();
    int tmpRow = getCol();
    Matrix retMatrix(retRow, retCol);

    for (int i = 0; i < retRow; i++)
    {
        for (int j = 0; j < retCol; j++)
        {
            double sum = 0;
            for (int k = 0; k < tmpRow; k++)
            {
                sum += getValue(i, k) * other.getValue(k, j);
            }
            retMatrix.setValue(i, j, sum);
        }
    }

    return retMatrix;
}

void Matrix::operator*=(const Matrix& other)
{
    assert(isSquared() && other.isSquared());
    assert(getCol() == other.getRow());

    int row = getRow();
    int col = getCol();
    int tmpRow = row;

    for (int i = 0; i < row; i++)
    {
        for (int j = 0; j < col; j++)
        {
            double sum = 0;
            for (int k = 0; k < tmpRow; k++)
            {
                sum += elem[i][k] * other.elem[k][j];
            }
            elem[i][j] = sum;
        }
    }
}

Matrix Matrix::operator/(const Matrix& other) const
{
    Matrix retMatrix(*this);
    retMatrix.pushStack(other, DIRECTION_HORIZONTAL);
    retMatrix.reducedEchelon();
    return retMatrix.getCols(getCol(), retMatrix.getCol());
}

Matrix Matrix::operator/(const double scaler) const
{
    assert(fabs(scaler - 0.0) > EPS);
    return this->operator*(1.0 / scaler);
}

bool Matrix::operator!=(const Matrix& other)
{
    return !(*this == other);
}

bool Matrix::operator==(const Matrix& other)
{
    if (this == &other)
    {
        return true;
    }

    if (getRow() != other.getRow() || getCol() != other.getCol())
    {
        return false;
    }

    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            if (fabs(elem[i][j] - other.elem[i][j]) > EPS)
            {
                return false;
            }
        }
    }

    return true;
}

void Matrix::resize(int row, int col)
{
    if (row == getRow() && col == getCol())
    {
        return;
    }

    elem.reserve(row);
    for (int r = 0; r < row; r++)
    {
        elem[r].reserve(col);
    }
}

Matrix Matrix::getRows(int startRow, int endRow)
{
    assert(startRow < endRow && endRow <= getRow());
    Matrix retMatrix(endRow - startRow, getCol());
    for (int r = startRow; r < endRow; r++)
    {
        retMatrix.elem[r - startRow] = elem[r];
    }
    return retMatrix;
}

Matrix Matrix::getCols(int startCol, int endCol)
{
    assert(startCol < endCol && endCol <= getCol());
    Matrix retMatrix(getRow(), endCol - startCol);
    for (int r = 0; r < getRow(); r++)
    {
        for (int c = startCol; c < endCol; c++)
        {
            retMatrix.elem[r][c - startCol] = elem[r][c];
        }
    }
    return retMatrix;
}

void Matrix::pushStack(const Matrix& other, Stack_Direction_E direction)
{
    switch (direction)
    {
    case DIRECTION_HORIZONTAL:
        pushHorizontalStack(other);
        break;
    case DIRECTION_VERTICAL:
        pushVerticalStack(other);
        break;
    default:
        break;
    }
}

void Matrix::pushHorizontalStack(const Matrix& other)
{
    assert(getRow() == other.getRow());
    int prevCol = getCol();

    for (int r = 0; r < getRow(); r++)
    {
        elem[r].insert(elem[r].end(), other.elem[r].begin(), other.elem[r].end());
    }

    assert(getCol() == prevCol + other.getCol());
}

void Matrix::pushVerticalStack(const Matrix& other)
{
    assert(getCol() == other.getCol());
    int prevRow = getRow();

    elem.insert(elem.end(), other.elem.begin(), other.elem.end());
    assert(prevRow + other.getRow() == getRow());
}

void Matrix::showMatrix() const
{
    printf("\n");
    for (int i = 0; i < getRow(); i++)
    {
        for (int j = 0; j < getCol(); j++)
        {
            printf("%15f", elem[i][j]);

        }
        printf("\n");
    }
    printf("\n");
}

void Matrix::showSize() const
{
    std::cout << getRow() << ", " << getCol() << std::endl;
}

Matrix Matrix::getSubMatrix(int startRow, int endRow, int startCol, int endCol) const
{
    assert(startRow < endRow);
    assert(startRow >= 0 && startRow < getRow());
    assert(endRow >= 0 && endRow <= getRow());

    assert(startCol < endCol);
    assert(startCol >= 0 && startCol < getCol());
    assert(endCol >= 0 && endCol <= getCol());

    Matrix subMatrix(endRow - startRow, endCol - startCol);
    for (int r = startRow; r < endRow; r++)
    {
        for (int c = startCol; c < endCol; c++)
        {
            subMatrix.elem[r - startRow][c - startCol] = elem[r][c];
        }
    }
    return subMatrix;
}

void Matrix::insert(const Matrix &matrix, int startRow, int endRow, int startCol, int endCol)
{
    assert(startRow < endRow);
    assert(startRow >= 0 && startRow < getRow());
    assert(endRow >= 0 && endRow <= getRow());
    assert(matrix.getRow() == endRow - startRow);

    assert(startCol < endCol);
    assert(startCol >= 0 && startCol < getCol());
    assert(endCol >= 0 && endCol <= getCol());
    assert(matrix.getCol() == endCol - startCol);

    for (int r = 0; r < matrix.getRow(); r++)
    {
        for (int c = 0; c < matrix.getCol(); c++)
        {
            elem[r + startRow][c + startCol] = matrix.elem[r][c];
        }
    }
}

bool Matrix::operator<(const Matrix &matrix) const
{
    if (getRow() < matrix.getRow())
    {
        return true;
    }
    else if (getRow() > matrix.getRow())
    {
        return false;
    }

    for (int r = 0; r < getRow(); r++)
    {
        if (elem[r][0] < matrix.elem[r][0])
        {
            return true;
        }
        else if (elem[r][0] > matrix.elem[r][0])
        {
            return false;
        }
    }
    return false;
}

double& Matrix::operator[](int posIndex)
{
    assert(posIndex >= 0 && posIndex < getRow());
    return elem[posIndex][0];
}

double Matrix::operator[](int posIndex) const
{
    assert(posIndex >= 0 && posIndex < getRow());
    return elem[posIndex][0];
}

bool Matrix::isSquared() const
{
    return getRow() == getCol();
}
