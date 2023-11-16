use crate::LinestringError;

#[test]
fn test_divide_into_shapes_1() -> Result<(), LinestringError> {
    let indices = vec![
        0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13,
        14, 14, 15, 16, 17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 24, 25, 25, 26,
        26, 27, 27, 28, 28, 29, 29, 30, 30, 31, 31, 32, 32, 16, 15, 0,
    ];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 2);
    assert_eq!(
        result[0],
        vec![0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 0]
    );
    assert_eq!(
        result[1],
        vec![16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 16]
    );
    Ok(())
}

#[test]
fn test_divide_into_shapes_2() -> Result<(), LinestringError> {
    let indices = vec![1, 2, 2, 5, 0, 2, 5, 4, 4, 3, 0, 3, 1, 6, 6, 7, 7, 3];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 3);
    assert_eq!(result[0], vec![3, 7, 6, 1, 2]);
    assert_eq!(result[1], vec![3, 0, 2]);
    assert_eq!(result[2], vec![3, 4, 5, 2]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_3() -> Result<(), LinestringError> {
    let indices = vec![
        1, 2, 2, 5, 0, 2, 5, 4, 4, 3, 0, 3, 1, 6, 6, 7, 7, 3, 10, 11, 11, 12,
    ];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 4);
    assert_eq!(result[0], vec![12, 11, 10]);
    assert_eq!(result[1], vec![3, 7, 6, 1, 2]);
    assert_eq!(result[2], vec![3, 0, 2]);
    assert_eq!(result[3], vec![3, 4, 5, 2]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_4() -> Result<(), LinestringError> {
    let indices = vec![
        1, 2, 2, 5, 0, 2, 5, 4, 4, 3, 0, 3, 1, 6, 6, 7, 7, 3, 10, 11, 11, 12, 12, 10,
    ];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 4);
    assert_eq!(result[0], vec![3, 7, 6, 1, 2]);
    assert_eq!(result[1], vec![3, 0, 2]);
    assert_eq!(result[2], vec![3, 4, 5, 2]);
    assert_eq!(result[3], vec![10, 11, 12, 10]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_5() -> Result<(), LinestringError> {
    let indices = vec![1, 2, 2, 4, 1, 0, 0, 3, 4, 1, 1, 3];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 2);
    assert_eq!(result[0], vec![1, 3, 0, 1]);
    assert_eq!(result[1], vec![1, 4, 2, 1]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_6() -> Result<(), LinestringError> {
    let indices = vec![1, 2, 2, 4, 1, 0, 0, 3, 4, 1, 1, 3, 11, 12, 12, 13];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 3);
    assert_eq!(result[0], vec![13, 12, 11]);
    assert_eq!(result[1], vec![1, 3, 0, 1]);
    assert_eq!(result[2], vec![1, 4, 2, 1]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_7() -> Result<(), LinestringError> {
    let indices = vec![0, 1, 3, 5, 1, 3, 3, 4, 1, 2];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 5);
    assert_eq!(result[0], vec![3, 4]);
    assert_eq!(result[1], vec![3, 5]);
    assert_eq!(result[2], vec![1, 2]);
    assert_eq!(result[3], vec![1, 3]);
    assert_eq!(result[4], vec![0, 1]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_8() -> Result<(), LinestringError> {
    let indices = vec![0, 1, 2, 3, 1, 4, 4, 5, 6, 7];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 3);
    assert_eq!(result[0], vec![6, 7]);
    assert_eq!(result[1], vec![5, 4, 1, 0]);
    assert_eq!(result[2], vec![2, 3]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_9() -> Result<(), LinestringError> {
    let indices = vec![3, 0, 0, 1, 1, 2, 2, 3, 2, 0, 1, 4, 0, 4];
    let result = super::divide_into_shapes(&indices);
    assert_eq!(result.len(), 5);
    assert_eq!(result[0], vec![2, 3, 0]);
    assert_eq!(result[1], vec![1, 4, 0]);
    assert_eq!(result[2], vec![1, 2]);
    assert_eq!(result[3], vec![0, 2]);
    assert_eq!(result[4], vec![0, 1]);
    Ok(())
}

#[test]
fn test_divide_into_shapes_10() -> Result<(), LinestringError> {
    // this will test that the duplicate edge 0-1 is removed
    let indices = vec![3, 0, 0, 1, 0, 1, 1, 2, 2, 3, 2, 0, 1, 4, 0, 4];
    let result = super::divide_into_shapes(&indices);
    //println!("result:{:?}", result);
    assert_eq!(result.len(), 5);
    assert_eq!(result[0], vec![2, 3, 0]);
    assert_eq!(result[1], vec![1, 4, 0]);
    assert_eq!(result[2], vec![1, 2]);
    assert_eq!(result[3], vec![0, 2]);
    assert_eq!(result[4], vec![0, 1]);
    Ok(())
}
