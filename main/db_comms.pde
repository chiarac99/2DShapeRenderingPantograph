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
  String sql = "INSERT INTO GuessesTable (time, shape_id, guess) VALUES (?, ?, ?)";
  try {
    PreparedStatement ps = db.prepareStatement(sql);
    ps.setString(1, str(millis()));
    ps.setInt(2, shapeId);
    ps.setString(3, guessText);
    ps.executeUpdate();
    ps.close();
  } catch (Exception e) {
    println("insertGuess failed: " + e.getMessage());
  }
}

void loadRecentGuesses(int shapeId) {
  String sql = "SELECT g.id, g.time, g.guess, s.shape_name " +
               "FROM GuessesTable g " +
               "JOIN ShapesTable s ON g.shape_id = s.id " +
               "WHERE g.shape_id = ? " +
               "ORDER BY g.id DESC " +
               "LIMIT 10";
  
  try {
    PreparedStatement ps = db.prepareStatement(sql);
    ps.setInt(1, shapeId);
    ResultSet rs = ps.executeQuery();
    
    while (rs.next()) {
      println(rs.getString("time") + " | shape: " + rs.getString("shape_name") + " | guess: " + rs.getString("guess"));
    }
    
    rs.close();
    ps.close();
  } catch (Exception e) {
    println("Query failed: " + e.getMessage());
  }
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
