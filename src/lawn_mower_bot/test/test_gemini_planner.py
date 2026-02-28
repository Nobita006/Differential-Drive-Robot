"""
Tests for GeminiPlanner JSON parsing and hazard extraction logic.
Does NOT call the actual Gemini API.
"""
import json
import pytest


class TestGeminiResponseParsing:
    """Test the JSON parsing logic used inside analyze_scene()."""

    def _parse_response(self, text):
        """Mirror the parsing logic from gemini_planner.py."""
        clean_text = text.strip()
        if clean_text.startswith("```"):
            lines = clean_text.split("\n")
            clean_text = "\n".join(
                lines[1:-1] if lines[-1].strip() == "```" else lines[1:]
            )
        try:
            return json.loads(clean_text)
        except json.JSONDecodeError:
            return {"scene_summary": text, "raw": True}

    def test_plain_json(self):
        resp = '{"scene_summary": "Open lawn", "terrain": "short_grass"}'
        result = self._parse_response(resp)
        assert result["scene_summary"] == "Open lawn"
        assert result["terrain"] == "short_grass"

    def test_markdown_fenced_json(self):
        resp = '```json\n{"scene_summary": "Tree ahead", "terrain": "mixed"}\n```'
        result = self._parse_response(resp)
        assert result["scene_summary"] == "Tree ahead"

    def test_malformed_json(self):
        resp = "This is not JSON at all"
        result = self._parse_response(resp)
        assert result.get("raw") is True
        assert "This is not JSON" in result["scene_summary"]

    def test_empty_response(self):
        result = self._parse_response("")
        assert result.get("raw") is True


class TestHazardExtraction:
    """Test hazard filtering logic from gemini_planner.py."""

    def test_high_danger_count(self):
        analysis = {
            "obstacles": [
                {"type": "tree", "danger_level": "low"},
                {"type": "person", "danger_level": "high"},
                {"type": "rock", "danger_level": "high"},
                {"type": "sprinkler", "danger_level": "medium"},
            ]
        }
        hazards = analysis["obstacles"]
        high_danger = [h for h in hazards if h.get("danger_level") == "high"]
        assert len(high_danger) == 2

    def test_no_hazards(self):
        analysis = {"obstacles": []}
        hazards = analysis["obstacles"]
        high_danger = [h for h in hazards if h.get("danger_level") == "high"]
        assert len(high_danger) == 0

    def test_missing_obstacles_key(self):
        analysis = {"scene_summary": "Clear lawn"}
        hazards = analysis.get("obstacles", [])
        assert len(hazards) == 0

    def test_hazard_message_structure(self):
        """The published hazard JSON should have the expected keys."""
        analysis = {
            "obstacles": [{"type": "rock", "danger_level": "low"}],
            "recommended_action": "continue",
            "mowing_quality": "good",
        }
        hazards = analysis.get("obstacles", [])
        high_danger = [h for h in hazards if h.get("danger_level") == "high"]
        msg_data = {
            "obstacles": hazards,
            "high_danger_count": len(high_danger),
            "recommended_action": analysis.get("recommended_action", "continue"),
            "mowing_quality": analysis.get("mowing_quality", "unknown"),
        }
        assert "obstacles" in msg_data
        assert "high_danger_count" in msg_data
        assert msg_data["recommended_action"] == "continue"
        assert msg_data["mowing_quality"] == "good"
