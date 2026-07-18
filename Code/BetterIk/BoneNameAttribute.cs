#nullable enable

using System;

namespace BetterIk;

/// <summary>Marks a string property as a bone name on the component's resolved
/// SkinnedModelRenderer. Purely a hint for the editor's control widget (see
/// Editor/BetterIk/BoneNameControlWidget.cs) to render a bone-picker dropdown instead of a
/// plain text box - has no effect on runtime behavior.</summary>
[AttributeUsage(AttributeTargets.Property)]
public sealed class BoneNameAttribute : Attribute
{
}
