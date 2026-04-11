export function zeros(rows, cols) {
  return Array.from({ length: rows }, () => Array(cols).fill(0));
}

export function identity(size) {
  const out = zeros(size, size);
  for (let i = 0; i < size; i += 1) {
    out[i][i] = 1;
  }
  return out;
}

export function diag(values) {
  const out = zeros(values.length, values.length);
  for (let i = 0; i < values.length; i += 1) {
    out[i][i] = values[i];
  }
  return out;
}

export function cloneMatrix(matrix) {
  return matrix.map((row) => [...row]);
}

export function addMatrices(a, b) {
  return a.map((row, i) => row.map((value, j) => value + b[i][j]));
}

export function subMatrices(a, b) {
  return a.map((row, i) => row.map((value, j) => value - b[i][j]));
}

export function scaleMatrix(matrix, scalar) {
  return matrix.map((row) => row.map((value) => value * scalar));
}

export function transpose(matrix) {
  return matrix[0].map((_, col) => matrix.map((row) => row[col]));
}

export function mulMatrices(a, b) {
  const out = zeros(a.length, b[0].length);
  for (let i = 0; i < a.length; i += 1) {
    for (let k = 0; k < b.length; k += 1) {
      for (let j = 0; j < b[0].length; j += 1) {
        out[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return out;
}

export function mulMatrixVector(matrix, vector) {
  return matrix.map((row) => row.reduce((sum, value, index) => sum + value * vector[index], 0));
}

export function addVectors(a, b) {
  return a.map((value, index) => value + b[index]);
}

export function subVectors(a, b) {
  return a.map((value, index) => value - b[index]);
}

export function scaleVector(vector, scalar) {
  return vector.map((value) => value * scalar);
}

export function dot(a, b) {
  return a.reduce((sum, value, index) => sum + value * b[index], 0);
}

export function magnitude(vector) {
  return Math.sqrt(dot(vector, vector));
}

export function inverse(matrix) {
  const size = matrix.length;
  const working = matrix.map((row, i) => [...row, ...identity(size)[i]]);

  for (let col = 0; col < size; col += 1) {
    let pivot = col;
    for (let row = col + 1; row < size; row += 1) {
      if (Math.abs(working[row][col]) > Math.abs(working[pivot][col])) {
        pivot = row;
      }
    }

    if (Math.abs(working[pivot][col]) < 1e-9) {
      throw new Error("Matrix is singular.");
    }

    if (pivot !== col) {
      [working[col], working[pivot]] = [working[pivot], working[col]];
    }

    const pivotValue = working[col][col];
    for (let j = 0; j < size * 2; j += 1) {
      working[col][j] /= pivotValue;
    }

    for (let row = 0; row < size; row += 1) {
      if (row === col) {
        continue;
      }
      const factor = working[row][col];
      for (let j = 0; j < size * 2; j += 1) {
        working[row][j] -= factor * working[col][j];
      }
    }
  }

  return working.map((row) => row.slice(size));
}

