def add_combined_legend(primary_axis, extra_axes=()):
    handles, labels = primary_axis.get_legend_handles_labels()
    for axis in extra_axes:
        extra_handles, extra_labels = axis.get_legend_handles_labels()
        handles.extend(extra_handles)
        labels.extend(extra_labels)
    return primary_axis.legend(handles, labels)