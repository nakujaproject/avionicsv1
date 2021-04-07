

void setup() {
    Serial.begin(115200);
    delay(300); // will pause Zero, Leonardo, etc until serial console opens
}

void loop() {
    float firstMatrix[2][3], secondMatrix[3][2], mul[2][2];
    firstMatrix[0][0] = 2.0;
    firstMatrix[0][1] = -3.0;
    firstMatrix[0][2] = 4.0;
    firstMatrix[1][0] = 53.0;
    firstMatrix[1][1] = 3.0;
    firstMatrix[1][2] = 5.0;
    firstMatrix[2][0] = 53.0;
    firstMatrix[2][1] = 3.0;
    firstMatrix[2][2] = 5.0;

    secondMatrix[0][0] = 3.0;
    secondMatrix[0][1] = 3.0;
    secondMatrix[0][2] = 3.0;
    secondMatrix[1][0] = 5.0;
    secondMatrix[1][1] = 0.0;
    secondMatrix[1][2] = 0.0;
    secondMatrix[2][0] = -3.0;
    secondMatrix[2][1] = 4.0;
    secondMatrix[2][2] = 4.0;
    
    // multiplyMatrices(firstMatrix, secondMatrix, mul, 2, 3, 3, 2);
    // displayt(mul, 2, 2);
    // float transp[3][2];

    // transpose(firstMatrix, transp);
    // displayt(transp, 3, 2);
    float sum[3][3];
    
    addition(firstMatrix, secondMatrix, sum);
    displayt(sum, 3, 3);

}


void multiplyMatrices(float firstMatrix[][3], float secondMatrix[][2], float mul[][2], int rowFirst, int columnFirst, int rowSecond, int columnSecond)
{
  int i, j, k;

  // Initializing elements of matrix mult to 0.
  for(i = 0; i < rowFirst; ++i)
  {
    for(j = 0; j < columnSecond; ++j)
    {
      mul[i][j] = 0;
    }
  }

  // Multiplying matrix firstMatrix and secondMatrix and storing in array mult.
  for(i = 0; i < rowFirst; ++i)
  {
    for(j = 0; j < columnSecond; ++j)
    {
      for(k=0; k<columnFirst; ++k)
      {
        mul[i][j] += firstMatrix[i][k] * secondMatrix[k][j];
      }
    }
  }
}

void displayt(float mult[][2], int rowFirst, int columnSecond)
{
  int i, j;
  Serial.println("\nOutput Matrix:\n");
  for(i = 0; i < rowFirst; ++i)
  {
    for(j = 0; j < columnSecond; ++j)
    {
      Serial.print(mult[i][j]);
      if(j == columnSecond - 1)
        Serial.println("\n\n");
    }
  }
}
float N1 = 3, N2 = 2;
// This function stores transpose of A[][] in B[][]
void transpose(float A[][3], float B[][2])
{
    int i, j;
    for (i = 0; i < 3; i++)
        for (j = 0; j < 2; j++)
            B[i][j] = A[j][i];
}


// function to add two 3x3 matrix
void addition(float m[3][3], float n[3][3], float sum[3][3])
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      sum[i][j] = m[i][j] + n[i][j];
}

// function to subtract two 3x3 matrix
void subtract(int m[3][3], int n[3][3], int result[3][3])
{
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      result[i][j] = m[i][j] - n[i][j];
}