import java.sql.*;

Connection db;

class ShapeRecord {
  int id;
  String shapeName, svgFilename, csvFilename;
  
  ShapeRecord(int id, String shapeName, String svgFilename, String csvFilename) {
    this.id = id;
    this.shapeName = shapeName;
    this.svgFilename = svgFilename;
    this.csvFilename = csvFilename;
  }
}

void dbConnect() {
  String dbPath = sketchPath("main.db");
  try {
    Class.forName("org.sqlite.JDBC");
    db = DriverManager.getConnection("jdbc:sqlite:" + dbPath);
    println("DB connected.");
  } catch (Exception e) {
    println("Connection failed: " + e.getMessage());
  }
}

void dbClose() {
  try {
    if (db != null) db.close();
  } catch (Exception e) {
    println("Error closing db: " + e.getMessage());
  }
}

void insertGuess(int shapeId, String guessText) {
  String sql = "INSERT INTO GuessesTable (time, shape_id, guess) VALUES (datetime('now'), ?, ?)";
  try {
    PreparedStatement ps = db.prepareStatement(sql);
    //ps.setString(1, str(millis()));
    ps.setInt(1, shapeId);
    ps.setString(2, guessText);
    ps.executeUpdate();
    ps.close();
  } catch (Exception e) {
    println("insertGuess failed: " + e.getMessage());
  }
}

String[] loadRecentGuesses(int shapeId) {
  String sql = "SELECT guess FROM GuessesTable WHERE shape_id = ? ORDER BY id DESC LIMIT 10";
  ArrayList<String> result = new ArrayList<String>();
  try {
    PreparedStatement ps = db.prepareStatement(sql);
    ps.setInt(1, shapeId);
    ResultSet rs = ps.executeQuery();
    while (rs.next()) result.add(rs.getString("guess"));
    rs.close(); ps.close();
  } catch (Exception e) {
    println("fetchRecentGuessStrings failed: " + e.getMessage());
  }
  return result.toArray(new String[0]);
}

ShapeRecord getCurrentShape(int shapeId) {
  String sql = "SELECT * FROM ShapesTable WHERE id = ?";
  
  try {
    PreparedStatement ps = db.prepareStatement(sql);
    ps.setInt(1, shapeId);
    ResultSet rs = ps.executeQuery();
    
    if (rs.next()) {
      ShapeRecord shape = new ShapeRecord(
        rs.getInt("id"),
        rs.getString("shape_name"),
        rs.getString("svg_filename"),
        rs.getString("csv_filename")
      );
      rs.close();
      ps.close();
      return shape;
    }
    
    rs.close();
    ps.close();
  } catch (Exception e) {
    println("getCurrentShape failed: " + e.getMessage());
  }
  
  return null;
}

String[][] getShapeList() {
  String sql = "SELECT id, shape_name FROM ShapesTable";
  ArrayList<String[]> rows = new ArrayList<String[]>();

  try {
    PreparedStatement ps = db.prepareStatement(sql);
    ResultSet rs = ps.executeQuery();

    while (rs.next()) {
      rows.add(new String[] { str(rs.getInt("id")), rs.getString("shape_name") });
    }

    rs.close();
    ps.close();
  } catch (Exception e) {
    println("getShapeList failed: " + e.getMessage());
  }

  return rows.toArray(new String[0][]);
}
