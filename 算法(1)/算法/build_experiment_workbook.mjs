import fs from "node:fs/promises";
import { fileURLToPath } from "node:url";
import { FileBlob, SpreadsheetFile, Workbook } from "@oai/artifact-tool";

const root = new URL(".", import.meta.url);
const manifestPath = new URL("experiments/run_manifest.json", root);
const outputPath = fileURLToPath(new URL("experiments/experiment_results.xlsx", root));
const manifest = JSON.parse(await fs.readFile(manifestPath, "utf8"));

const workbook = Workbook.create();
const resultsSheet = workbook.worksheets.add("实验结果");
const notesSheet = workbook.worksheets.add("说明");

const headers = [
  "实验编号", "时间窗口(分钟)", "备用电池(组)", "允许反向", "状态", "全任务可行",
  "解类型", "总任务数", "完成任务数", "完成率(%)", "完成任务编号", "未完成任务编号",
  "无人机-任务对应", "完工时间(分钟)", "换电次数", "无人机活动时间(分钟)",
  "车辆运行时间(分钟)", "最终电量(%)", "求解耗时(ms)", "扩展标签数", "生成标签数",
  "校验通过", "校验项数",
];

const rows = manifest.map((row) => [
  row.experiment_id,
  row.time_horizon_min,
  row.spare_battery_sets,
  row.allow_reverse ? "是" : "否",
  row.status,
  row.feasible ? "是" : "否",
  row.solution_kind,
  row.total_task_count,
  row.completed_task_count,
  row.completion_rate_pct,
  row.completed_task_ids.join(";"),
  row.uncompleted_task_ids.join(";"),
  row.uav_task_mapping.join(";"),
  row.finish_time_min,
  row.swaps_used,
  row.drone_active_time_min,
  row.vehicle_travel_time_min,
  row.current_soc_pct,
  row.runtime_ms,
  row.expanded_labels,
  row.generated_labels,
  row.validation_passed ? "是" : "否",
  row.validation_check_count,
]);

resultsSheet.showGridLines = false;
resultsSheet.getRange("A1:W1").merge();
resultsSheet.getRange("A1").values = [["昌平线无人机—移动机巢协同巡检实验结果"]];
resultsSheet.getRange("A2:W2").merge();
resultsSheet.getRange("A2").values = [["5项任务，时间窗口120/125/135/150分钟，备用电池2/3/4/5组"]];
resultsSheet.getRange("A4:W4").values = [headers];
resultsSheet.getRange("A5").write(rows);
resultsSheet.getRange("A1:W20").format.font = { name: "Arial", size: 10 };
resultsSheet.getRange("A1").format.font = { name: "Arial", size: 15, bold: true, color: "#1F2937" };
resultsSheet.getRange("A2").format.font = { name: "Arial", size: 10, italic: true, color: "#4B5563" };
resultsSheet.getRange("A4:W4").format = {
  fill: "#1F4E78",
  font: { name: "Arial", size: 10, bold: true, color: "#FFFFFF" },
  horizontalAlignment: "center",
  verticalAlignment: "center",
  wrapText: true,
};
resultsSheet.getRange("A5:W20").format.verticalAlignment = "center";
resultsSheet.getRange("B5:J20").format.horizontalAlignment = "center";
resultsSheet.getRange("N5:W20").format.horizontalAlignment = "center";
resultsSheet.getRange("J5:J20").format.numberFormat = "0.0";
resultsSheet.getRange("S5:S20").format.numberFormat = "0.000";
resultsSheet.getRange("A4:W20").format.borders = { preset: "inside", style: "thin", color: "#D9E2F3" };
resultsSheet.getRange("A4:W20").format.autofitColumns();
resultsSheet.getRange("A4:W20").format.autofitRows();
resultsSheet.getRange("A:A").format.columnWidth = 14;
resultsSheet.getRange("E:E").format.columnWidth = 52;
resultsSheet.getRange("E5:E20").format.wrapText = true;
resultsSheet.getRange("K:L").format.columnWidth = 26;
resultsSheet.getRange("M:M").format.columnWidth = 42;
resultsSheet.freezePanes.freezeRows(4);
resultsSheet.tables.add("A4:W20", true, "ExperimentResultsTable");
resultsSheet.getRange("F5:F20").conditionalFormats.add("containsText", {
  text: "是",
  format: { fill: "#E2F0D9", font: { color: "#375623", bold: true } },
});
resultsSheet.getRange("F5:F20").conditionalFormats.add("containsText", {
  text: "否",
  format: { fill: "#FCE4D6", font: { color: "#843C0C" } },
});

notesSheet.showGridLines = false;
notesSheet.getRange("A1:B1").merge();
notesSheet.getRange("A1").values = [["实验说明"]];
notesSheet.getRange("A3:B10").values = [
  ["项目", "内容"],
  ["线路范围", "昌平西山口—南邵，6站5区间"],
  ["任务规模", "5项巡检任务"],
  ["资源规模", "1辆移动机巢、1架无人机U01"],
  ["时间步长", "5分钟"],
  ["敏感性参数", "时间窗口120/125/135/150分钟；备用电池2/3/4/5组"],
  ["截止时间调整", "同步调整原本等于基础总时域的任务截止时间"],
  ["可行性含义", "全任务可行表示完成5项任务并到达指定终点"],
];
notesSheet.getRange("A12:B16").values = [
  ["主要结论", "结果"],
  ["120分钟", "最多完成4项任务，增加到4或5组电池仍无法完成T05"],
  ["125分钟", "最多完成4项任务"],
  ["首次可行组合", "135分钟、4组备用电池；完工135分钟，实际换电4次"],
  ["150分钟", "4组或5组电池均可行，最优完工时间仍为135分钟"],
];
notesSheet.getRange("A1:D16").format.font = { name: "Arial", size: 10 };
notesSheet.getRange("A1").format.font = { name: "Arial", size: 15, bold: true, color: "#1F2937" };
for (const headerRange of ["A3:B3", "A12:B12"]) {
  notesSheet.getRange(headerRange).format = {
    fill: "#1F4E78",
    font: { name: "Arial", size: 10, bold: true, color: "#FFFFFF" },
  };
}
notesSheet.getRange("A3:B10").format.borders = { preset: "inside", style: "thin", color: "#D9E2F3" };
notesSheet.getRange("A12:B16").format.borders = { preset: "inside", style: "thin", color: "#D9E2F3" };
notesSheet.getRange("A3:B16").format.wrapText = true;
notesSheet.getRange("A:A").format.columnWidth = 18;
notesSheet.getRange("B:B").format.columnWidth = 72;
notesSheet.getRange("A1:B16").format.autofitRows();

const resultsTableCheck = await workbook.inspect({
  kind: "table",
  range: "实验结果!A1:W20",
  include: "values,formulas",
  tableMaxRows: 20,
  tableMaxCols: 23,
});
console.log(resultsTableCheck.ndjson);

const formulaErrorCheck = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!",
  options: { useRegex: true, maxResults: 100 },
  summary: "final formula error scan",
});
console.log(formulaErrorCheck.ndjson);

const preview = await workbook.render({
  sheetName: "实验结果",
  range: "A1:W20",
  scale: 1,
  format: "png",
});
await fs.writeFile(new URL("experiments/experiment_results_preview.png", root), new Uint8Array(await preview.arrayBuffer()));

const notesPreview = await workbook.render({
  sheetName: "说明",
  range: "A1:B16",
  scale: 1.5,
  format: "png",
});
await fs.writeFile(new URL("experiments/experiment_notes_preview.png", root), new Uint8Array(await notesPreview.arrayBuffer()));

const output = await SpreadsheetFile.exportXlsx(workbook);
await output.save(outputPath);

const savedWorkbook = await SpreadsheetFile.importXlsx(await FileBlob.load(outputPath));
const savedCheck = await savedWorkbook.inspect({
  kind: "table",
  range: "实验结果!A4:W20",
  include: "values,formulas",
  tableMaxRows: 20,
  tableMaxCols: 23,
});
console.log(savedCheck.ndjson);
